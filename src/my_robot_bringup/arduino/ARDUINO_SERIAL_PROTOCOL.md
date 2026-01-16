# Arduino Serial Port Protokolü

Bu dokümantasyon, ROS2 ile Arduino arasındaki seri port haberleşme protokolünü açıklar. Protokol, geçen senenin `ROSArduinoBridge` kodlarına dayanmaktadır.

## Genel Bilgiler

- **Baud Rate**: 57600 (varsayılan, yapılandırılabilir)
- **Parity**: None
- **Data Bits**: 8
- **Stop Bits**: 1
- **Komut Formatı**: Tek karakter komut + argümanlar + `\r` (carriage return)
- **Yanıt Formatı**: ASCII string + `\n` (newline)

## Komutlar

### Motor Kontrolü

**Komut**: `m left_ticks right_ticks\r`

Motor hızlarını ayarlar. `left_ticks` ve `right_ticks` değerleri, Arduino üzerindeki PID controller'ın hedef tick/frame değerleridir.

**Örnek**:
```
m 10 -10\r
```

**Yanıt**: `OK\n`

---

### Encoder Okuma

**Komut**: `e\r`

Mevcut encoder değerlerini okur.

**Yanıt**: `left_ticks right_ticks\n`

**Örnek**:
```
e\r
→ 15234 -15234\n
```

---

### Encoder Reset

**Komut**: `r\r`

Encoder sayaçlarını sıfırlar.

**Yanıt**: `OK\n`

---

### Buzzer Kontrolü

**Komut**: `b state\r`

- `state`: `0` (kapalı) veya `1` (açık)

**Örnek**:
```
b 1\r  → Buzzer açık
b 0\r  → Buzzer kapalı
```

**Yanıt**: `OK\n`

---

### Batarya Okuma

**Komut**: `f\r`

Batarya voltajını ve yüzde değerini okur.

**Yanıt**: `voltage:percent\n`

**Örnek**:
```
f\r
→ 12.34:85.2\n
```

---

### PID Parametre Güncelleme

**Komut**: `u Kp:Kd:Ki:Ko\r`

PID controller parametrelerini günceller.

**Parametreler**:
- `Kp`: Proportional gain
- `Kd`: Derivative gain
- `Ki`: Integral gain
- `Ko`: Output scaling factor

**Örnek**:
```
u 20:12:0:50\r
```

**Yanıt**: `OK\n`

---

## ROS2 Entegrasyonu

### `arduino_bridge` Node

`arduino_bridge` node'u aşağıdaki işlevleri sağlar:

1. **Subscriber**: `/cmd_vel` (geometry_msgs/Twist)
   - Twist mesajını alır
   - Differential drive kinematiği ile motor komutlarına çevirir
   - Arduino'ya `m` komutu gönderir

2. **Publisher**: `/odom` (nav_msgs/Odometry)
   - Encoder verilerini periyodik olarak okur (`e` komutu)
   - Odometry hesaplar ve yayınlar

3. **Auto-stop**: 
   - 2 saniye boyunca `/cmd_vel` gelmezse motorları durdurur

### Parametreler

```yaml
serial_port: "/dev/ttyUSB0"          # Serial port cihazı
baud_rate: 57600                      # Baud rate
timeout: 1.0                          # Timeout (saniye)
wheel_separation: 0.30                # Tekerlekler arası mesafe (m)
wheel_radius: 0.05                    # Tekerlek yarıçapı (m)
encoder_counts_per_rev: 3600          # Encoder çözünürlüğü
loop_rate: 30.0                       # Odometry güncelleme hızı (Hz)
odom_frame: "odom"                    # Odometry frame ID
base_frame: "base_footprint"          # Base frame ID
publish_odom_tf: false                # TF yayınla (odom_tf_broadcaster kullanılıyorsa false)
```

### Launch Dosyası

```bash
ros2 launch my_robot_bringup arduino_robot_bringup.launch.py serial_port:=/dev/ttyUSB0
```

## Arduino Kod Örneği

Arduino tarafında `ROSArduinoBridge.ino` benzeri bir kod kullanılmalıdır. Ana komut işleme yapısı:

```cpp
void loop() {
  while (Serial.available() > 0) {
    chr = Serial.read();
    
    if (chr == 13) {  // CR (carriage return)
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // ... parsing logic ...
  }
  // PID loop, encoder reading, etc.
}
```

## Hata Ayıklama

### Seri Port Bulunamıyor

```bash
# Kullanılabilir serial portları listele
ls -l /dev/ttyUSB* /dev/ttyACM*

# Kullanıcıyı dialout grubuna ekle (seri port erişimi için)
sudo usermod -a -G dialout $USER
# Logout/login gerekli
```

### Bağlantı Sorunları

1. Baud rate'in eşleştiğinden emin olun (varsayılan: 57600)
2. Arduino'nun doğru port'a bağlı olduğunu kontrol edin
3. Başka bir programın port'u kullanmadığından emin olun
4. Arduino'nun başlatılması için yeterli süre bekleyin (2 saniye)

### Encoder Verileri Gelmüyor

- Encoder bağlantılarını kontrol edin
- Encoder interrupt pin'lerini doğru yapılandırın
- `encoder_counts_per_rev` parametresinin doğru olduğundan emin olun

## Referanslar

- `rosardunio-main/ROSArduinoBridge/` - Geçen senenin Arduino kodu
- `teknofest-agv-main/diffdrive_arduino/` - ROS2 Control entegrasyonu örneği
