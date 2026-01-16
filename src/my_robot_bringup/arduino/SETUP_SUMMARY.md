# Arduino Mega 2560 - Motor ve Encoder Kurulum Özeti

## ✅ Mevcut Durum

Sistem **Arduino Mega 2560** için **2 adet BTS7960 motor sürücü** ve **2 adet encoder** ile yapılandırılmıştır.

## 🔧 Aktif Konfigürasyon

### ROSArduinoBridge.ino içinde:

```cpp
#define USE_BASE              // Motor ve encoder kontrolü aktif
#define ARDUINO_ENC_COUNTER   // Arduino interrupt tabanlı encoder okuma
#define BTS7960_MOTOR_DRIVER  // BTS7960 motor sürücü kullanımı
```

## 📋 Bağlantı Özeti

### Sol Motor + Encoder
- **Motor Sürücü:** BTS7960 #1
  - Pin 6 → R_PWM (Geri)
  - Pin 9 → L_PWM (İleri)
- **Encoder:**
  - Pin 2 → Channel A (INT0)
  - Pin 3 → Channel B (INT1)

### Sağ Motor + Encoder
- **Motor Sürücü:** BTS7960 #2
  - Pin 10 → R_PWM (Geri)
  - Pin 11 → L_PWM (İleri)
- **Encoder:**
  - Pin 18 → Channel A (INT5) ⚠️ Mega'ya özgü
  - Pin 19 → Channel B (INT4) ⚠️ Mega'ya özgü

## ✅ Kod Durumu

| Özellik | Durum | Dosya |
|---------|-------|-------|
| **BTS7960 Motor Sürücü** | ✅ Aktif | `motor_driver.h`, `motor_driver.ino` |
| **2 Encoder (Interrupt)** | ✅ Aktif | `encoder_driver.h`, `encoder_driver.ino` |
| **PID Kontrolü** | ✅ Aktif | `diff_controller.h` |
| **ROSArduinoBridge Protokolü** | ✅ Aktif | `ROSArduinoBridge.ino` |

## 📚 Detaylı Dokümantasyon

1. **Bağlantı Şeması:** `WIRING_DIAGRAM.md` - Tüm pin bağlantıları
2. **Arduino Mega Setup:** `ARDUINO_MEGA_SETUP.md` - Mega'ya özgü ayarlar
3. **Serial Protokol:** `ARDUINO_SERIAL_PROTOCOL.md` - Komut protokolü
4. **Firmware README:** `firmware/README.md` - Kurulum talimatları

## 🎯 Teknofest-AGV ile Uyumluluk

✅ **Aynı yapı:**
- 2x BTS7960 motor sürücü
- 2x Encoder (her motora 1 adet)
- Arduino Mega 2560
- ROSArduinoBridge protokolü
- PID kontrolü

## 🚀 Kullanıma Hazır

Kod **hazır durumda**. Sadece:
1. Arduino Mega 2560'a yükleyin
2. Bağlantıları yapın (WIRING_DIAGRAM.md'ye göre)
3. ROS2'den bağlanın

**Test komutları:**
```bash
# Serial Monitor'da (57600 baud)
e          # Encoder değerleri
m 100 100  # Motor test (ileri)
m -100 -100 # Motor test (geri)
```

## ⚠️ Önemli Notlar

1. **Arduino Mega 2560 gerekli** - Pin 18 ve 19 interrupt pin'leri sadece Mega'da var
2. **Common ground** - Arduino GND ile motor güç kaynağı GND birleştirilmeli
3. **Ayrı güç kaynağı** - Motorlar için 12V-24V ayrı kaynak gerekli
4. **Baud rate:** 57600 (Arduino ve ROS2'de aynı olmalı)

---

**Sistem Teknofest-AGV ile aynı yapıda ve kullanıma hazır!** 🎉
