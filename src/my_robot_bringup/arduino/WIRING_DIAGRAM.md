# Arduino Mega 2560 - BTS7960 Motor Sürücü ve Encoder Bağlantı Şeması

## 📋 Genel Bakış

Bu dokümanda Arduino Mega 2560'ın 2 adet BTS7960 motor sürücüsü ve 2 adet encoder ile bağlantısı açıklanmıştır.

**Yapı:**
- 1x Arduino Mega 2560
- 2x BTS7960 Motor Sürücü (Sol ve Sağ motor için)
- 2x Encoder (Her motora 1 adet)

---

## 🔌 Motor Sürücü Bağlantıları (BTS7960)

### Sol Motor (BTS7960 #1)

```
Arduino Mega 2560    →    BTS7960 Motor Sürücü
─────────────────────────────────────────────
Pin 6  (PWM)         →    R_PWM (Geri yön PWM)
Pin 9  (PWM)         →    L_PWM (İleri yön PWM)
GND                  →    GND
5V (opsiyonel)       →    VCC (eğer logic seviyesi gerekiyorsa)

BTS7960 Motor Sürücü →    Sol Motor
─────────────────────────────────────────────
M+                   →    Motor + terminali
M-                   →    Motor - terminali
VM+                  →    Motor güç kaynağı + (12V-24V)
VM-                  →    Motor güç kaynağı -
L_EN                 →    VCC (Enable - sürekli aktif)
R_EN                 →    VCC (Enable - sürekli aktif)
```

### Sağ Motor (BTS7960 #2)

```
Arduino Mega 2560    →    BTS7960 Motor Sürücü
─────────────────────────────────────────────
Pin 10 (PWM)         →    R_PWM (Geri yön PWM)
Pin 11 (PWM)         →    L_PWM (İleri yön PWM)
GND                  →    GND
5V (opsiyonel)       →    VCC (eğer logic seviyesi gerekiyorsa)

BTS7960 Motor Sürücü →    Sağ Motor
─────────────────────────────────────────────
M+                   →    Motor + terminali
M-                   →    Motor - terminali
VM+                  →    Motor güç kaynağı + (12V-24V)
VM-                  →    Motor güç kaynağı -
L_EN                 →    VCC (Enable - sürekli aktif)
R_EN                 →    VCC (Enable - sürekli aktif)
```

**Önemli Notlar:**
- L_EN ve R_EN pinleri donanımsal olarak VCC'ye bağlı (kodda kontrol edilmiyor)
- Motor güç kaynağı (VM+, VM-) Arduino'dan ayrı olmalı (12V-24V)
- Arduino GND ile motor güç kaynağı GND'yi birleştirin (common ground)

---

## 📊 Encoder Bağlantıları

### Sol Encoder

```
Encoder              →    Arduino Mega 2560
─────────────────────────────────────────────
Channel A           →    Pin 2  (INT0 - Interrupt pin)
Channel B           →    Pin 3  (INT1 - Interrupt pin)
VCC                 →    5V (veya 3.3V - encoder'a göre)
GND                 →    GND
```

### Sağ Encoder

```
Encoder              →    Arduino Mega 2560
─────────────────────────────────────────────
Channel A           →    Pin 18 (INT5 - Interrupt pin) ⚠️ Mega'ya özgü!
Channel B           →    Pin 19 (INT4 - Interrupt pin) ⚠️ Mega'ya özgü!
VCC                 →    5V (veya 3.3V - encoder'a göre)
GND                 →    GND
```

**Önemli Notlar:**
- Pin 18 ve 19 **sadece Arduino Mega 2560'da interrupt pin'leridir**
- Arduino Uno/Nano'da bu pin'ler interrupt özelliği yok
- Encoder'lar interrupt kullanarak yüksek hassasiyetle okunur

---

## 🔋 Güç Bağlantıları

```
Güç Kaynağı 1 (Motor için):
─────────────────────────────
12V-24V +           →    BTS7960 #1 VM+ (Sol motor)
12V-24V +           →    BTS7960 #2 VM+ (Sağ motor)
12V-24V -           →    BTS7960 #1 VM- (Sol motor)
12V-24V -           →    BTS7960 #2 VM- (Sağ motor)
12V-24V -           →    Arduino GND (Common ground)

Güç Kaynağı 2 (Arduino için):
─────────────────────────────
USB veya 7-12V DC   →    Arduino VIN
GND                 →    Arduino GND
```

**Önemli:**
- Motor güç kaynağı Arduino'dan **ayrı** olmalı
- Mutlaka **common ground** bağlantısı yapın (Arduino GND ↔ Motor GND)
- BTS7960'ların L_EN ve R_EN pinleri VCC'ye bağlı (sürekli aktif)

---

## 📐 Pin Özet Tablosu

| Arduino Pin | Bağlantı | Açıklama |
|-------------|----------|----------|
| **Pin 2** | Sol Encoder A | INT0 (Interrupt) |
| **Pin 3** | Sol Encoder B | INT1 (Interrupt) |
| **Pin 6** | Sol Motor Backward (R_PWM) | PWM - Geri yön |
| **Pin 9** | Sol Motor Forward (L_PWM) | PWM - İleri yön |
| **Pin 10** | Sağ Motor Backward (R_PWM) | PWM - Geri yön |
| **Pin 11** | Sağ Motor Forward (L_PWM) | PWM - İleri yön |
| **Pin 18** | Sağ Encoder A | INT5 (Interrupt) - Mega'ya özgü |
| **Pin 19** | Sağ Encoder B | INT4 (Interrupt) - Mega'ya özgü |
| **GND** | Tüm GND'ler | Common ground |
| **5V** | Encoder VCC | Encoder güç kaynağı |

---

## ✅ Doğrulama Adımları

### 1. Motor Testi

Arduino Serial Monitor'da (57600 baud):

```
m 100 100    # Her iki motor ileri (düşük hız)
m -100 -100  # Her iki motor geri (düşük hız)
m 0 0        # Motorları durdur
```

**Beklenen:** Motorlar dönmeli (yön ve hız kontrolü çalışmalı)

### 2. Encoder Testi

Arduino Serial Monitor'da:

```
e            # Encoder değerlerini oku
```

**Beklenen:** `left_ticks right_ticks` formatında değerler gelmeli

Motorları manuel olarak döndürdüğünüzde encoder değerleri değişmeli.

### 3. PID Testi

Motor komutu gönderildiğinde PID controller encoder değerlerini okuyup motor hızını ayarlamalı.

---

## ⚠️ Yaygın Hatalar

### Motor Çalışmıyor

1. **Güç bağlantısını kontrol edin:**
   - Motor güç kaynağı bağlı mı? (12V-24V)
   - L_EN ve R_EN VCC'ye bağlı mı?
   - Common ground bağlı mı?

2. **PWM pin'lerini kontrol edin:**
   - Pin 6, 9, 10, 11 doğru bağlı mı?
   - PWM sinyali geliyor mu? (multimetre ile ölçün)

### Encoder Çalışmıyor

1. **Pin bağlantılarını kontrol edin:**
   - Pin 2, 3, 18, 19 doğru bağlı mı?
   - Encoder VCC ve GND bağlı mı?

2. **Interrupt pin'lerini kontrol edin:**
   - Pin 18 ve 19 **sadece Mega'da** interrupt pin'idir
   - Arduino Uno/Nano kullanıyorsanız çalışmaz

### Serial Haberleşme Sorunu

1. **Baud rate kontrolü:**
   - Arduino: 57600
   - ROS2: 57600
   - Serial Monitor: 57600

2. **Port kontrolü:**
   - Doğru port seçili mi? (`/dev/ttyUSB0` veya `/dev/ttyACM0`)
   - Başka bir program port'u kullanıyor mu?

---

## 📚 Referanslar

- **Firmware kodu:** `firmware/ROSArduinoBridge.ino`
- **Motor driver:** `firmware/motor_driver.h` ve `motor_driver.ino`
- **Encoder driver:** `firmware/encoder_driver.h` ve `encoder_driver.ino`
- **Protokol:** `ARDUINO_SERIAL_PROTOCOL.md`
- **Mega setup:** `ARDUINO_MEGA_SETUP.md`

---

## 🎯 Sonuç

Bu bağlantı şeması ile:
- ✅ 2 adet BTS7960 motor sürücü çalışır
- ✅ 2 adet encoder çalışır (her motora 1 adet)
- ✅ PID kontrolü aktif
- ✅ ROS2 ile haberleşme sağlanır

**Sistem Teknofest-AGV projesi ile aynı yapıdadır.**
