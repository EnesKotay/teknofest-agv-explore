# Yapay Zeka İçin Arduino Haberleşme Sistemi Açıklama Prompt'u

Aşağıdaki prompt'u kopyalayıp yapay zekaya yapıştırabilirsiniz:

---

## PROMPT (Kopyala-Yapıştır için hazır):

```
Merhaba! Bana ROS2 tabanlı bir AGV robot projesindeki Arduino haberleşme sistemini detaylı olarak açıklayabilir misin? 

PROJE BAĞLAMI:
- Teknofest AGV yarışması için geliştirilmiş otonom keşif robotu
- ROS2 Jazzy kullanılıyor
- Robot: Differential drive (2 tekerlekli), LIDAR sensörü, SLAM, Nav2 navigasyon

DONANIM YAPISI:
- Arduino Mega 2560 (mikrodenetleyici)
- 2x BTS7960 Motor Sürücü (sol ve sağ motor için ayrı)
- 2x Encoder (her motora 1 adet, quadrature encoder)
- Serial port üzerinden USB ile haberleşme

HABERLEŞME PROTOKOLÜ:
ROSArduinoBridge protokolü kullanılıyor. Bu protokol tek karakter komutlar + argümanlar + carriage return (\r) formatında çalışıyor.

Temel Komutlar:
1. Motor Kontrolü: "m left_ticks right_ticks\r"
   - left_ticks: Sol motor için PID controller'a gönderilecek tick/frame değeri (integer)
   - right_ticks: Sağ motor için PID controller'a gönderilecek tick/frame değeri (integer)
   - Yanıt: "OK\n" veya boş

2. Encoder Okuma: "e\r"
   - Yanıt: "left_ticks right_ticks\n" (boşluk ile ayrılmış iki integer)

3. Encoder Reset: "r\r"
   - Yanıt: "OK\n"

4. PID Parametreleri: "u Kp:Kd:Ki:Ko\r"
   - Kp, Kd, Ki, Ko: PID controller parametreleri (integer)

5. Batarya Okuma: "f\r"
   - Yanıt: "voltage:percent\n" (örn: "12.34:85.2\n")

6. Buzzer Kontrolü: "b state\r"
   - state: 0 (kapalı) veya 1 (açık)
   - Yanıt: "OK\n"

BAUD RATE: 57600
Data Format: 8 bit, no parity, 1 stop bit

YAZILIM MİMARİSİ:

1. ARDUINO FIRMWARE (Arduino Mega'da çalışan kod):
   - ROSArduinoBridge.ino: Ana program döngüsü
   - motor_driver.h/.ino: BTS7960 motor sürücü kontrolü (PWM sinyalleri)
   - encoder_driver.h/.ino: Encoder okuma (interrupt tabanlı, quadrature decoding)
   - diff_controller.h: PID kontrol algoritması (30 Hz döngü)
   - Serial komut parsing ve yürütme

   Arduino'nun yaptığı işler (ROSArduinoBridge.ino - MEVCUT KOD):
   - setup(): Serial.begin(57600), encoder pin'lerini INPUT_PULLUP olarak ayarlar (Pin 2, 3, 18, 19), interrupt'ları bağlar (attachInterrupt), motor controller'ı başlatır (initMotorController), PID'i resetler (resetPID())
   - loop(): Serial.available() kontrolü, komut parsing (commands.h), PID hesaplama (30 Hz - PID_INTERVAL = 1000/30 ms), encoder okuma (interrupt-based), auto-stop kontrolü (2 saniye - AUTO_STOP_INTERVAL), buzzer güncelleme, DFPlayer kontrolü, servo kontrolü
   - Komut işleme: 'e' (encoder okuma), 'm' (motor kontrolü), 'r' (encoder reset), 'u' (PID parametreleri), 'f' (batarya), 'b' (buzzer) vb.
   - PID controller: 30 Hz döngüde çalışır, encoder feedback ile motor hızını kontrol eder
   - Encoder okuma: Interrupt tabanlı (leftEncoderAInterrupt, leftEncoderBInterrupt, rightEncoderAInterrupt, rightEncoderBInterrupt), quadrature decoding

2. ROS2 PYTHON BRIDGE NODE (arduino_bridge.py - MEVCUT KOD):

   Ana Sınıf: ArduinoBridge(Node)
   - pyserial kütüphanesi ile Arduino'ya bağlanır (serial.Serial)
   - Thread-safe: threading.Lock kullanır (serial_lock)
   
   Başlatma (__init__):
   - Parametreler: serial_port, baud_rate, timeout, wheel_separation, wheel_radius, encoder_counts_per_rev, loop_rate (30 Hz), odom_frame, base_frame
   - Serial port açılır: serial.Serial(port, baudrate=57600, timeout=1.0, bytesize=8, parity=NONE, stopbits=1)
   - Buffer temizlenir: flushInput(), flushOutput()
   - 2 saniye beklenir (Arduino başlatma için)
   - Encoder'lar resetlenir: reset_encoders() fonksiyonu çağrılır
   - Publisher: /odom (Odometry)
   - Subscriber: /cmd_vel (Twist)
   - Timer: update_odometry() - 30 Hz (1.0/loop_rate)

   Fonksiyonlar:
   
   a) reset_encoders():
      - RESET_ENCODERS = b'r\r' komutu gönderir
      - "OK" yanıtı bekler
   
   b) read_encoders() -> (left_ticks, right_ticks):
      - READ_ENCODERS = b'e\r' komutu gönderir
      - Yanıt: "left_ticks right_ticks\n" formatında
      - split() ile parse edilir, int'e çevrilir
      - Hata durumunda (None, None) döner
   
   c) send_motor_command(left_ticks, right_ticks):
      - Format: f"m {int(left_ticks)} {int(right_ticks)}\r".encode('ascii')
      - Serial port'a yazılır (thread-safe lock ile)
   
   d) cmd_vel_callback(msg: Twist):
      - linear_vel = msg.linear.x
      - angular_vel = msg.angular.z
      - Differential drive kinematiği:
        v_left = linear_vel - (angular_vel * wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * wheel_separation / 2.0)
      - m/s -> rad/s:
        omega_left = v_left / wheel_radius
        omega_right = v_right / wheel_radius
      - rad/s -> ticks/frame (PID için):
        pid_rate = 30.0
        ticks_per_frame_left = omega_left * encoder_counts_per_rev / (2.0 * π * pid_rate)
        ticks_per_frame_right = omega_right * encoder_counts_per_rev / (2.0 * π * pid_rate)
      - send_motor_command() çağrılır
      - Auto-stop timer sıfırlanır: last_cmd_time = clock.now()
   
   e) update_odometry() (Timer callback - 30 Hz):
      - read_encoders() çağrılır
      - Delta time hesaplanır (ROS2 clock kullanılır)
      - Encoder farkları:
        d_left = left_ticks - encoder_left_prev
        d_right = right_ticks - encoder_right_prev
      - Ticks -> metrik mesafe:
        meters_per_tick = (2.0 * π * wheel_radius) / encoder_counts_per_rev
        d_left_m = d_left * meters_per_tick
        d_right_m = d_right * meters_per_tick
      - Odometry hesaplama:
        d_center = (d_left_m + d_right_m) / 2.0
        d_theta = (d_right_m - d_left_m) / wheel_separation
        theta += d_theta
        x += d_center * cos(theta)
        y += d_center * sin(theta)
        theta = atan2(sin(theta), cos(theta))  # Normalizasyon
      - Hız hesaplama:
        v_left = d_left_m / dt
        v_right = d_right_m / dt
        linear_vel = (v_left + v_right) / 2.0
        angular_vel = (v_right - v_left) / wheel_separation
      - Odometry mesajı oluşturulur:
        - position: x, y, z=0
        - orientation: quaternion_from_euler(0, 0, theta)
        - twist: linear_vel (x), angular_vel (z)
        - covariance: [0.01, 0.01, 0.01, 0.01, 0.01, 0.1] (pose), [0.01, 0.01, ..., 0.1] (twist)
      - /odom yayınlanır
      - Auto-stop kontrolü:
        Eğer 2 saniye boyunca /cmd_vel gelmezse: send_motor_command(0, 0)
   
   f) quaternion_from_euler(roll, pitch, yaw) (static method):
      - Euler açılarından quaternion'a dönüşüm (ROS2 standart formülü)

3. ODOM TF BROADCASTER (odom_tf_broadcaster.py - MEVCUT KOD):
   - /odom topic'ini dinler
   - TF yayınlar: odom -> base_footprint
   - odom_tf_broadcaster node'u olarak çalışır
   - real_robot_bringup.launch.py içinde başlatılır

4. LAUNCH DOSYASI (real_robot_bringup.launch.py - MEVCUT KOD):
   - arduino_bridge node'u başlatır (parametrelerle)
   - robot_state_publisher başlatır (URDF -> TF)
   - odom_tf_broadcaster başlatır
   - Parametreler: serial_port, baud_rate, wheel_separation, wheel_radius, encoder_counts_per_rev

VERİ AKIŞI (MEVCUT SİSTEM):

1. Navigasyon Komutları:
   Nav2 Stack → /cmd_vel (Twist) → arduino_bridge.cmd_vel_callback() → send_motor_command() → Serial.write("m left right\r") → Arduino loop() → runCommand('m') → setMotorSpeeds() → PWM sinyalleri → Motorlar

2. Odometry Geri Beslemesi:
   Arduino (encoder interrupt) → readEncoder() → Serial.print("left right\n") → arduino_bridge.read_encoders() → update_odometry() (30 Hz timer) → Odometry mesajı → /odom (publish) → Nav2 Stack + SLAM
   
3. TF Yayınlama:
   /odom (Odometry) → odom_tf_broadcaster → TF (odom -> base_footprint) → TF tree

4. Zamanlama (MEVCUT KOD):
   - Motor komutları: Event-driven (her /cmd_vel mesajı geldiğinde cmd_vel_callback() çağrılır)
   - Encoder okuma: 30 Hz timer-based (update_odometry() timer callback)
   - PID loop: Arduino'da 30 Hz (PID_INTERVAL = 1000/30 ms)
   - Auto-stop kontrolü: update_odometry() içinde, 2 saniye (auto_stop_interval)

KONFİGÜRASYON PARAMETRELERİ (MEVCUT KOD):
- serial_port: "/dev/ttyUSB0" (varsayılan, launch ile değiştirilebilir)
- baud_rate: 57600 (hem Arduino hem ROS2'de)
- timeout: 1.0 (saniye, serial okuma timeout)
- wheel_separation: 0.30 (metre, varsayılan - launch ile değiştirilebilir)
- wheel_radius: 0.05 (metre, varsayılan - launch ile değiştirilebilir)
- encoder_counts_per_rev: 3600 (varsayılan - launch ile değiştirilebilir)
- loop_rate: 30.0 (Hz, odometry güncelleme frekansı - Arduino PID_RATE ile aynı)
- odom_frame: "odom" (ROS2 frame ID)
- base_frame: "base_footprint" (ROS2 frame ID)
- publish_odom_tf: False (odom_tf_broadcaster kullanıldığı için)

GÜVENLİK ÖZELLİKLERİ:
- Auto-stop: 2 saniye boyunca /cmd_vel gelmezse motorlar otomatik durur
- Thread-safe: Serial port erişimi threading.Lock ile korunur
- Timeout: Serial okuma işlemleri için timeout değeri (1 saniye)

SORUMLULUKLARIN AYRILMASI:
- Arduino: Düşük seviye motor kontrolü, encoder okuma, PID kontrolü
- ROS2 Bridge: Yüksek seviye kinematik dönüşümler, odometry hesaplama, ROS2 entegrasyonu
- Nav2: Yüksek seviye navigasyon ve path planning

Bu sistemin nasıl çalıştığını adım adım açıklayabilir misin? Özellikle:
1. Bir navigasyon komutunun Arduino'ya nasıl ulaştığını
2. Encoder verilerinin nasıl okunup odometry'ye dönüştürüldüğünü
3. PID controller'ın motor kontrolündeki rolünü
4. Differential drive kinematiğinin nasıl uygulandığını

detaylı bir şekilde anlatır mısın?
```

---

## Kısa Versiyon (Daha Özet):

```
ROS2 tabanlı AGV robot projesinde Arduino Mega 2560 ile haberleşme sistemi nasıl çalışır?

Donanım: Arduino Mega + 2x BTS7960 motor sürücü + 2x encoder
Protokol: ROSArduinoBridge (57600 baud, "m left right\r", "e\r" vb. komutlar)
Yazılım: Python bridge node (arduino_bridge.py) - /cmd_vel → motor, encoder → /odom

Bu sistemin çalışma prensibini, veri akışını ve komut formatlarını detaylı açıklayabilir misin?
```

---

## Özel Konular İçin Ayrı Prompt'lar:

### 1. Sadece Protokol İçin:
```
ROSArduinoBridge serial protokolünü açıkla. Komut formatları, yanıt formatları, baud rate ve kullanım senaryolarını detaylandır.
```

### 2. Sadece Kinematik Dönüşümler İçin:
```
Differential drive robotta /cmd_vel (Twist) mesajının motor komutlarına nasıl dönüştürüldüğünü ve encoder verilerinin odometry'ye nasıl çevrildiğini matematiksel formüllerle açıkla.
```

### 3. Sadece PID Kontrolü İçin:
```
Arduino'da PID controller'ın motor hız kontrolündeki rolünü, encoder feedback ile ilişkisini ve 30 Hz döngüde nasıl çalıştığını açıkla.
```

---

## Kullanım Notları:

1. **Tam Prompt:** Tüm sistemi anlamak için → Uzun versiyonu kullanın
2. **Hızlı Genel Bakış:** Kısa versiyonu kullanın
3. **Belirli Konu:** İlgili özel prompt'u kullanın

**İpucu:** Prompt'a ek olarak şunları da ekleyebilirsiniz:
- "Türkçe olarak açıkla"
- "Diagrams/şemalarla göster"
- "Kod örnekleri ver"
- "Yaygın hataları ve çözümlerini de ekle"
