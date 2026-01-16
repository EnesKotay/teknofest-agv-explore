# Arduino Mega 2560 Uyumluluk Dokümantasyonu

## ✅ Arduino Mega 2560 İçin Özel Ayarlanmış

Haberleşme sistemi ve firmware **Arduino Mega 2560** için özel olarak ayarlanmıştır.

## 🔌 Pin Tanımlamaları (Arduino Mega 2560)

### Encoder Pin'leri (Mega'ya Özgü!)

```cpp
// encoder_driver.h içinde
// Sol encoder - Digital pin 2 ve 3 (INT0 ve INT1)
#define LEFT_ENC_PIN_A 2   // Pin 2 - INT0
#define LEFT_ENC_PIN_B 3   // Pin 3 - INT1

// Sağ encoder - Digital pin 18 ve 19 (INT5 ve INT4) 
// ⚠️ BUNLAR ARDUINO MEGA'YA ÖZGÜ! (Uno/Nano'da yok)
#define RIGHT_ENC_PIN_A 18  // Pin 18 - INT5
#define RIGHT_ENC_PIN_B 19  // Pin 19 - INT4
```

**Önemli**: Pin 18 ve 19 **Arduino Mega 2560'da interrupt pin'leri**dir. Arduino Uno/Nano'da bu pin'ler interrupt özelliği yoktur.

### Motor Sürücü Pin'leri (BTS7960)

```cpp
// motor_driver.h içinde
// Sol Motor
#define LEFT_MOTOR_FORWARD   9   // L_PWM pin
#define LEFT_MOTOR_BACKWARD  6   // R_PWM pin

// Sağ Motor  
#define RIGHT_MOTOR_FORWARD  11   // L_PWM pin
#define RIGHT_MOTOR_BACKWARD 10   // R_PWM pin
```

### Diğer Pin'ler

```cpp
// ROSArduinoBridge.ino içinde
#define BUZZER_PIN     8      // Buzzer
#define EMERGENCY_BUTTON_PIN  14  // Emergency button
#define DFPLAYER_RX_PIN 12    // DFPlayer Mini RX
#define DFPLAYER_TX_PIN 13    // DFPlayer Mini TX
#define BATTERY_PIN    A0     // Batarya voltaj ölçümü
```

## 📊 Arduino Modelleri Karşılaştırması

| Özellik | Arduino Mega 2560 | Arduino Uno/Nano | Durum |
|---------|-------------------|------------------|-------|
| **Interrupt Pin Sayısı** | 6 (2, 3, 18, 19, 20, 21) | 2 (2, 3) | ✅ Mega gerekli |
| **Pin 18 (INT5)** | ✅ Var | ❌ Yok | ✅ Mega gerekli |
| **Pin 19 (INT4)** | ✅ Var | ❌ Yok | ✅ Mega gerekli |
| **RAM** | 8 KB | 2 KB | ✅ Mega daha iyi |
| **Flash** | 256 KB | 32 KB | ✅ Mega daha iyi |
| **Digital Pin Sayısı** | 54 | 14 | ✅ Mega yeterli |

## ⚠️ Arduino Uno/Nano Kullanılamaz!

**Neden?**
- Sağ encoder için Pin 18 ve 19 interrupt pin'leri gerekiyor
- Arduino Uno/Nano'da sadece Pin 2 ve 3 interrupt pin'i var
- Pin 18 ve 19 Uno/Nano'da interrupt özelliği yok

**Çözüm (Uno/Nano kullanmak istiyorsanız):**
1. Sadece **tek encoder** kullanın (sol tekerlek)
2. Sağ encoder'ı farklı bir pin'e taşıyın (ama interrupt olmayacak, güvenilirlik düşer)
3. **Arduino Mega 2560 kullanın** (ÖNERİLEN)

## 🔧 Bağlantı Şeması

### Arduino Mega 2560 → BTS7960 Motor Sürücü

```
Mega Pin 6  → Left Motor Backward (R_PWM)
Mega Pin 9  → Left Motor Forward (L_PWM)
Mega Pin 10 → Right Motor Backward (R_PWM)
Mega Pin 11 → Right Motor Forward (L_PWM)
```

### Arduino Mega 2560 → Encoder'lar

```
Mega Pin 2  → Left Encoder A (INT0)
Mega Pin 3  → Left Encoder B (INT1)
Mega Pin 18 → Right Encoder A (INT5) ⚠️ Mega'ya özgü!
Mega Pin 19 → Right Encoder B (INT4) ⚠️ Mega'ya özgü!
```

## 📝 Baud Rate ve Serial Ayarları

```cpp
#define BAUDRATE 57600  // Serial port baud rate
```

**ROS2 tarafında** da aynı baud rate kullanılmalı:

```yaml
# real_robot_bringup.launch.py
baud_rate: 57600
```

## ✅ Doğrulama

Arduino IDE'de yüklerken:

1. **Tools → Board → Arduino Mega 2560** seçilmeli
2. Serial port doğru seçilmeli
3. Yükleme başarılı olmalı

**Test (Serial Monitor - 57600 baud):**

```
e        # Encoder değerleri okunmalı
f        # Batarya voltajı okunmalı
m 10 -10 # Motor test
```

## 🎯 Sonuç

**Sistem %100 Arduino Mega 2560 için ayarlanmıştır.**

- ✅ Encoder pin'leri (18, 19) Mega'ya özgü
- ✅ Interrupt kullanımı Mega ile uyumlu
- ✅ Pin sayısı yeterli (54 digital pin)
- ✅ RAM ve Flash yeterli

**Arduino Uno/Nano kullanılamaz** - Pin 18 ve 19 interrupt özelliği yok.
