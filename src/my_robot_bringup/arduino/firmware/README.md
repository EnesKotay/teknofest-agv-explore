# Arduino Firmware - ROSArduinoBridge

Bu klasör, ROS2 ile haberleşmek için gerekli Arduino firmware kodlarını içerir.

## 📁 Dosya Yapısı

- **ROSArduinoBridge.ino** - Ana Arduino sketch dosyası
- **commands.h** - Serial komut tanımlamaları
- **motor_driver.h/.ino** - Motor sürücü fonksiyonları (BTS7960 destekli)
- **encoder_driver.h/.ino** - Encoder okuma fonksiyonları
- **diff_controller.h** - PID kontrol algoritması
- **sensors.h** - Sensör fonksiyonları (Ultrasonic, vb.)
- **servos.h/.ino** - Servo motor kontrol fonksiyonları

## 🔧 Kurulum

### Gereksinimler

1. **Arduino IDE** (1.8.x veya 2.x)
2. **Kütüphaneler:**
   - `Servo` (Arduino built-in)
   - `SoftwareSerial` (Arduino built-in)
   - `DFRobotDFPlayerMini` (DFPlayer kullanılıyorsa) - Library Manager'dan yüklenebilir

### Yükleme Adımları

1. Arduino IDE'yi açın
2. `ROSArduinoBridge.ino` dosyasını açın
3. Gerekli kütüphaneleri yükleyin:
   ```
   Tools -> Manage Libraries -> "DFRobotDFPlayerMini" ara ve yükle
   ```
4. Arduino modelini seçin (Arduino Mega 2560 önerilir)
5. Serial port'u seçin
6. **Upload** butonuna tıklayın

## ⚙️ Konfigürasyon

### Motor Sürücü

Varsayılan: **BTS7960 Motor Driver**

```cpp
#define BTS7960_MOTOR_DRIVER
```

Pin tanımlamaları (`motor_driver.h`):
- Sol Motor: Pin 9 (FORWARD), Pin 6 (BACKWARD)
- Sağ Motor: Pin 11 (FORWARD), Pin 10 (BACKWARD)

### Encoder

Varsayılan: **Arduino Encoder Counter**

```cpp
#define ARDUINO_ENC_COUNTER
```

Pin tanımlamaları (`encoder_driver.h`):
- Sol Encoder: Pin 2 (A), Pin 3 (B)
- Sağ Encoder: Pin 18 (A), Pin 19 (B)

### Baud Rate

Varsayılan: **57600**

```cpp
#define BAUDRATE 57600
```

### Özellikler (İsteğe Bağlı)

- **Servo Motor**: `#define USE_SERVOS` - Pin 4'e bağlı
- **DFPlayer Mini**: `#define USE_DFPLAYER` - Pin 12/13 (RX/TX)
- **Emergency Button**: Pin 14'e bağlı
- **Buzzer**: Pin 8'e bağlı

## 📡 Protokol

ROSArduinoBridge protokolü kullanılır. Detaylar için `../ARDUINO_SERIAL_PROTOCOL.md` dosyasına bakın.

### Temel Komutlar

- `e\r` - Encoder değerlerini oku
- `m left_ticks right_ticks\r` - Motor hızlarını ayarla
- `r\r` - Encoder'ları sıfırla
- `b 1\r` / `b 0\r` - Buzzer aç/kapa
- `f\r` - Batarya voltajını oku

## 🔍 Test

Arduino IDE Serial Monitor'da (57600 baud):

```
e        # Encoder değerleri
f        # Batarya voltajı
z        # Baud rate
m 10 -10 # Motor test (sol ileri, sağ geri)
```

## 🐛 Sorun Giderme

### Encoder Çalışmıyor

- Pin bağlantılarını kontrol edin (2, 3, 18, 19)
- Interrupt pin'lerinin doğru kullanıldığından emin olun
- Encoder güç bağlantısını kontrol edin

### Motor Çalışmıyor

- Motor sürücü güç bağlantısını kontrol edin
- Enable pin'lerin doğru bağlandığından emin olun
- PWM pin'lerinin doğru bağlandığından emin olun (6, 9, 10, 11)

### Serial Haberleşme Sorunu

- Baud rate'in eşleştiğinden emin olun (57600)
- Başka bir programın port'u kullanmadığından emin olun
- USB kablosunun veri aktarımını desteklediğinden emin olun

## 📚 Referanslar

- Geçen senenin kodları: `rosardunio-main/ROSArduinoBridge/`
- ROS2 Bridge: `../../my_robot_explore/arduino_bridge.py`
- Protokol dokümantasyonu: `../ARDUINO_SERIAL_PROTOCOL.md`
