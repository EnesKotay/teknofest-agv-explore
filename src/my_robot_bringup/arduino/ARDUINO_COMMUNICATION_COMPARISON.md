# Arduino Haberleşme Karşılaştırması - Teknofest-AGV vs Bizim Sistem

## 📊 Genel Karşılaştırma

| Özellik | Teknofest-AGV | Bizim Sistem | Durum |
|---------|---------------|--------------|-------|
| **Haberleşme Protokolü** | ROSArduinoBridge | ROSArduinoBridge | ✅ Aynı |
| **Baud Rate** | 57600 | 57600 | ✅ Aynı |
| **Motor Komutu** | `m left right\r` | `m left right\r` | ✅ Aynı |
| **Encoder Komutu** | `e\r` | `e\r` | ✅ Aynı |
| **PID Komutu** | `u Kp:Kd:Ki:Ko\r` | `u Kp:Kd:Ki:Ko\r` | ✅ Aynı |
| **Implementation** | ROS2 Control (C++) | Python Bridge | ⚠️ Farklı |

## 🔧 Protokol Detayları

### Teknofest-AGV (diffdrive_arduino)

**Hardware Interface:** ROS2 Control (C++)
- `arduino_comms.hpp` - LibSerial kullanır
- `read_encoder_values()` - `"e\r"` komutu
- `set_motor_values()` - `"m left right\r"` komutu
- `set_pid_values()` - `"u Kp:Kd:Ki:Ko\r"` komutu

**Launch:** ROS2 Control node + controller spawner

### Bizim Sistem

**Bridge Node:** Python (`arduino_bridge.py`)
- `pyserial` kullanır
- Aynı komutlar: `e\r`, `m left right\r`, `u Kp:Kd:Ki:Ko\r`
- `/cmd_vel` → Motor komutları
- Encoder → `/odom`

**Ayrıca:** ROS2 Control hardware interface (C++) mevcut ama henüz kullanılmıyor

## ✅ Protokol Uyumluluğu

### 1. Motor Kontrolü

**Her ikisinde de:**
```
Komut: "m left_ticks right_ticks\r"
Yanıt: "OK\n" (veya boş)
```

**Bizim kod:**
```python
# arduino_bridge.py
def cmd_vel_callback(self, msg):
    # Twist -> motor ticks dönüşümü
    left_ticks = ...
    right_ticks = ...
    cmd = f"m {left_ticks} {right_ticks}\r"
    self.serial_conn.write(cmd.encode())
```

**Teknofest-AGV (C++):**
```cpp
// arduino_comms.hpp
void set_motor_values(int val_1, int val_2) {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
}
```

✅ **Aynı protokol**

---

### 2. Encoder Okuma

**Her ikisinde de:**
```
Komut: "e\r"
Yanıt: "left_ticks right_ticks\n"
```

**Bizim kod:**
```python
# arduino_bridge.py
response = self.send_command(READ_ENCODERS)  # "e\r"
left_enc_str, right_enc_str = response.split()
left_encoder = int(left_enc_str)
right_encoder = int(right_enc_str)
```

**Teknofest-AGV (C++):**
```cpp
// arduino_comms.hpp
void read_encoder_values(int &val_1, int &val_2) {
    std::string response = send_msg("e\r");
    // Parse "left right\n"
    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
}
```

✅ **Aynı protokol**

---

### 3. PID Parametreleri

**Her ikisinde de:**
```
Komut: "u Kp:Kd:Ki:Ko\r"
Yanıt: "OK\n"
```

**Bizim kod:**
```python
# (Henüz kullanılmıyor ama protokol mevcut)
```

**Teknofest-AGV (C++):**
```cpp
// arduino_comms.hpp
void set_pid_values(int k_p, int k_d, int k_i, int k_o) {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
}
```

✅ **Aynı protokol (bizde kullanılmıyor ama destekleniyor)**

---

## 🔄 Mevcut Durum

### Şu Ana Kadar Geldiklerimiz:

1. ✅ **Arduino Firmware:** ROSArduinoBridge protokolü ile uyumlu
2. ✅ **Python Bridge:** `arduino_bridge.py` - protokol uyumlu
3. ✅ **C++ Hardware Interface:** `my_robot_hardware` - protokol uyumlu
4. ✅ **Serial Protokol:** Aynı komut formatları
5. ✅ **Launch Dosyaları:** Python bridge ile çalışıyor

### Eksikler (Teknofest-AGV'de var):

1. ❌ **ROS2 Control Launch:** C++ hardware interface'i kullanmak için launch dosyası yok (ama gerekli değil, Python bridge çalışıyor)
2. ❌ **Buzzer Kontrolü:** Hardware interface'de var ama Python bridge'de yok (ihtiyaç olursa eklenebilir)
3. ❌ **Farklı Encoder Çözünürlükleri:** Teknofest-AGV'de `left_enc_counts_per_rev` ve `right_enc_counts_per_rev` ayrı (bizde tek `enc_counts_per_rev`)

---

## 🎯 Sonuç

### Protokol Uyumluluğu: ✅ %100 UYUMLU

**Her iki sistem de aynı Arduino protokolünü kullanıyor:**
- Motor komutları: `m left right\r`
- Encoder okuma: `e\r`
- PID ayarları: `u Kp:Kd:Ki:Ko\r`
- Baud rate: 57600

**Fark:** Implementation (Python vs C++ ROS2 Control) ama protokol aynı.

### Kullanım

**Şu an aktif olan:**
- ✅ Python `arduino_bridge.py` - Çalışıyor, Teknofest-AGV ile uyumlu protokol kullanıyor

**Alternatif (kullanılabilir ama henüz kullanılmıyor):**
- ✅ C++ `my_robot_hardware` - ROS2 Control ile kullanılabilir, aynı protokol

**Öneri:** Mevcut Python bridge'i kullanmaya devam edin. Protokol %100 uyumlu, ekstra bir şey yapmanıza gerek yok.

---

## 📚 Referanslar

- **Teknofest-AGV:** `/home/enes/Desktop/teknofest-agv-main/diffdrive_arduino/`
- **Bizim Arduino Firmware:** `src/my_robot_bringup/arduino/firmware/`
- **Bizim Python Bridge:** `src/my_robot_explore/my_robot_explore/arduino_bridge.py`
- **Bizim C++ Hardware:** `src/my_robot_hardware/`
- **Protokol Dokümantasyonu:** `src/my_robot_bringup/arduino/ARDUINO_SERIAL_PROTOCOL.md`
