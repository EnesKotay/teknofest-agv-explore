# 🤖 Robot Kurulum Rehberi - Donanım ve Yazılım

## 📋 İçindekiler
1. [Donanım Bağlantıları](#1-donanım-bağlantıları)
2. [Arduino Firmware Yükleme](#2-arduino-firmware-yükleme)
3. [Raspberry Pi Kurulumu](#3-raspberry-pi-kurulumu)
4. [PC Kurulumu (Geliştirme)](#4-pc-kurulumu-geliştirme)
5. [Sistem Testi](#5-sistem-testi)
6. [Robot Çalıştırma](#6-robot-çalıştırma)

---

## 1. Donanım Bağlantıları

### 🔌 Arduino Mega 2560 Bağlantıları

#### Motor Sürücüleri (BTS7960) - Sol Motor
| BTS7960 | Arduino Mega Pin | Açıklama |
|---------|------------------|----------|
| RPWM | Pin 9 | İleri yön PWM |
| LPWM | Pin 6 | Geri yön PWM |
| R_EN | 5V | Enable (her zaman aktif) |
| L_EN | 5V | Enable (her zaman aktif) |
| VCC | 12V | Motor güç kaynağı |
| GND | GND | Toprak |

#### Motor Sürücüleri (BTS7960) - Sağ Motor
| BTS7960 | Arduino Mega Pin | Açıklama |
|---------|------------------|----------|
| RPWM | Pin 11 | İleri yön PWM |
| LPWM | Pin 10 | Geri yön PWM |
| R_EN | 5V | Enable (her zaman aktif) |
| L_EN | 5V | Enable (her zaman aktif) |
| VCC | 12V | Motor güç kaynağı |
| GND | GND | Toprak |

#### Encoder Bağlantıları
| Encoder | Arduino Mega Pin | Açıklama |
|---------|------------------|----------|
| Sol Encoder A | Pin 2 | Interrupt pin |
| Sol Encoder B | Pin 3 | Interrupt pin |
| Sağ Encoder A | Pin 18 | Interrupt pin |
| Sağ Encoder B | Pin 19 | Interrupt pin |
| Encoder VCC | 5V | Güç (motor sürücüsü ile ortak olabilir) |
| Encoder GND | GND | Toprak |

#### Diğer Bağlantılar
| Bileşen | Arduino Mega Pin | Açıklama |
|---------|------------------|----------|
| Buzzer | Pin 8 | Ses çıkışı |
| Emergency Button | Pin 14 | Acil durdurma butonu |
| DFPlayer Mini RX | Pin 13 | Ses modülü (opsiyonel) |
| DFPlayer Mini TX | Pin 12 | Ses modülü (opsiyonel) |

### 🔌 Raspberry Pi Bağlantıları

| Bileşen | Raspberry Pi Bağlantısı | Açıklama |
|---------|-------------------------|----------|
| Arduino Mega | USB Port | `/dev/ttyUSB0` veya `/dev/ttyACM0` |
| RPLIDAR A2 | USB Port | `/dev/ttyUSB1` (Arduino farklı portta ise) |
| Güç | 5V/3A USB-C | Raspberry Pi 5 için |

### ⚠️ ÖNEMLİ NOTLAR

1. **Güç Kaynağı**: 
   - Motorlar için ayrı 12V güç kaynağı kullanın
   - Arduino ve Raspberry Pi için ayrı güç kaynakları önerilir
   - Toprakları (GND) ortaklayın

2. **Encoder VCC**: 
   - Encoder VCC'yi motor sürücüsü VCC ile ortaklamak genellikle sorun yaratmaz
   - Ancak gürültü varsa ayrı 5V kaynak kullanın

3. **USB Port Kontrolü**:
   ```bash
   # Raspberry Pi'de port kontrolü
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

---

## 2. Arduino Firmware Yükleme

### 📦 Gerekli Arduino Kütüphaneleri

Arduino IDE'de şu kütüphaneleri yükleyin:
- **DFRobotDFPlayerMini** (DFPlayer kullanıyorsanız)
- **Servo** (Servo motor kullanıyorsanız)

### 📝 Firmware Yükleme Adımları

1. **Arduino IDE'yi açın** (1.8.x veya 2.x)

2. **Board seçin**:
   - Tools → Board → Arduino AVR Boards → Arduino Mega 2560

3. **Port seçin**:
   - Tools → Port → Arduino Mega'nın bağlı olduğu port

4. **Firmware dosyasını açın**:
   ```
   src/my_robot_bringup/arduino/firmware/ROSArduinoBridge.ino
   ```

5. **Ayarları kontrol edin**:
   - `BAUDRATE = 57600` (Line 19)
   - `USE_BASE` tanımlı olmalı (Line 6)
   - Diğer özellikler (DFPlayer, Servo) ihtiyaca göre

6. **Yükleyin**:
   - Sketch → Upload (Ctrl+U)

7. **Serial Monitor'da test edin** (57600 baud):
   ```
   e    # Encoder okuma
   m 0 0  # Motor durdurma
   z    # Baud rate kontrolü
   ```

---

## 3. Raspberry Pi Kurulumu

### 📦 Gerekli Paketler

```bash
# Sistem güncellemesi
sudo apt update && sudo apt upgrade -y

# ROS2 Jazzy kurulumu (eğer yoksa)
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

# Gerekli kütüphaneler
sudo apt install -y \
    libserial-dev \
    python3-serial \
    python3-pip \
    git \
    build-essential \
    cmake

# Dialout grubuna ekle (serial port erişimi için)
sudo usermod -a -G dialout $USER
# Çıkış yapıp tekrar giriş yapın veya:
newgrp dialout
```

### 🔧 Workspace Kurulumu

```bash
# Workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Projeyi klonla (veya kopyala)
git clone <your-repo-url> teknofest-agv-explore
# veya
# scp -r /path/to/my_robot_explore-main1 raspi@<raspi-ip>:~/ros2_ws/src/teknofest-agv-explore

cd ~/ros2_ws

# Bağımlılıkları yükle
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

### 🔌 Serial Port İzinleri

```bash
# Port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# İzinleri kontrol et (crw-rw---- ve dialout grubu olmalı)
# Eğer root:root ise:
sudo chmod 666 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0

# Kalıcı çözüm için udev kuralı (opsiyonel)
sudo nano /etc/udev/rules.d/99-arduino.rules
# İçine ekle:
# SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", MODE="0666", GROUP="dialout"
# (Arduino'nun vendor ID'sini bulmak için: lsusb)
```

### 📡 LIDAR Kurulumu

```bash
# RPLIDAR ROS2 paketi (eğer yoksa)
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros2.git
cd ~/ros2_ws
colcon build --packages-select rplidar_ros
```

---

## 4. PC Kurulumu (Geliştirme)

### 📦 Gerekli Paketler

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install -y \
    libserial-dev \
    python3-serial \
    git \
    build-essential \
    cmake \
    python3-colcon-common-extensions

# ROS2 Jazzy (eğer yoksa)
# https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html
```

### 🔧 Workspace Kurulumu

```bash
# Workspace oluştur
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Projeyi klonla
git clone <your-repo-url> my_robot_explore-main1

cd ~/ros2_ws

# Bağımlılıkları yükle
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

---

## 5. Sistem Testi

### 🧪 Arduino Testi

```bash
# Serial port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# Miniterm ile test (Raspberry Pi'de)
sudo apt install miniterm
miniterm /dev/ttyUSB0 57600

# Komutlar:
# e<Enter>     -> Encoder değerleri
# m 100 100<Enter>  -> Motor testi (dikkatli!)
# m 0 0<Enter> -> Motor durdur
# z<Enter>     -> Baud rate kontrolü
```

### 🧪 ROS2 Control Testi

```bash
# Source
source ~/ros2_ws/install/setup.bash

# ROS2 Control launch (sadece motor kontrolü)
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0

# Başka terminalde:
# Encoder değerlerini kontrol et
ros2 topic echo /joint_states

# Motor testi (dikkatli!)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

### 🧪 LIDAR Testi

```bash
# LIDAR node'u başlat
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB1 \
    -p serial_baudrate:=256000 \
    -p frame_id:=laser_link

# Scan verilerini kontrol et
ros2 topic echo /scan
```

---

## 6. Robot Çalıştırma

### ⚠️ KRİTİK UYARI: İki Kontrol Sistemi Çakışması

**ASLA `arduino_bridge.py` ve ROS2 Control'i aynı anda çalıştırmayın!**

Her ikisi de Arduino'ya komut göndermeye çalışırsa:
- ❌ Serial port çakışması
- ❌ Motor kontrolü karışır
- ❌ Encoder okuma hataları
- ❌ Robot "iki beyinli" duruma düşer

**Kullanım Senaryoları:**

1. **ROS2 Control (ÖNERİLEN)**: Modern, standart yöntem
   ```bash
   ros2 launch my_robot_bringup real_robot_ros2_control.launch.py
   ```

2. **arduino_bridge.py (ESKİ)**: Sadece test için, ROS2 Control yoksa
   ```bash
   ros2 launch my_robot_bringup real_robot_bringup.launch.py
   ```

**`real_robot_exploration.launch.py` ROS2 Control kullanır, `arduino_bridge.py` kullanmaz!**

---

### 🚀 Tam Sistem (Otonom Keşif) - ROS2 Control ile

```bash
# Raspberry Pi'de
source ~/ros2_ws/install/setup.bash

# Tam sistem başlat (SLAM + Nav2 + Exploration)
# NOT: Bu launch ROS2 Control kullanır, arduino_bridge.py KULLANMAZ
ros2 launch my_robot_bringup real_robot_exploration.launch.py \
    serial_port:=/dev/ttyUSB0 \
    lidar_port:=/dev/ttyUSB1 \
    use_rviz:=false  # Headless için

# Farklı portlar için:
ros2 launch my_robot_bringup real_robot_exploration.launch.py \
    serial_port:=/dev/ttyACM0 \
    lidar_port:=/dev/ttyUSB0
```

### 🚀 Sadece ROS2 Control (Motor Kontrolü + Odometry)

```bash
# Motor kontrolü ve odometry (arduino_bridge.py YOK)
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### 🚀 Manuel Kontrol (arduino_bridge.py ile - SADECE TEST İÇİN)

```bash
# ⚠️ UYARI: Bu launch ROS2 Control KULLANMAZ, sadece arduino_bridge.py kullanır
# ROS2 Control ile BİRLİKTE ÇALIŞTIRMAYIN!
ros2 launch my_robot_bringup real_robot_bringup.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=57600
```

### 📊 Launch Dosyası Karşılaştırması

| Launch Dosyası | Motor Kontrolü | Odometry | Kullanım |
|----------------|----------------|----------|----------|
| `real_robot_ros2_control.launch.py` | ROS2 Control | ROS2 Control | ✅ Önerilen |
| `real_robot_exploration.launch.py` | ROS2 Control | ROS2 Control | ✅ Otonom keşif |
| `real_robot_bringup.launch.py` | arduino_bridge.py | arduino_bridge.py | ⚠️ Sadece test |

### 🎮 Manuel Motor Kontrolü

```bash
# Teleop (başka terminalde)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# veya cmd_vel pub
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

---

## 🔍 Sorun Giderme

### ❌ Serial Port Bulunamıyor

```bash
# Port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# Dialout grubunda mı?
groups $USER

# İzinleri düzelt
sudo chmod 666 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0
```

### ❌ libserial Bulunamıyor

```bash
# Kurulum
sudo apt install libserial-dev

# Build tekrar
cd ~/ros2_ws
colcon build --packages-select my_robot_hardware
```

### ❌ Encoder Değerleri 0

- Encoder bağlantılarını kontrol edin
- Interrupt pin'lerini kontrol edin (2, 3, 18, 19)
- Arduino Serial Monitor'da `e` komutu ile test edin

### ❌ Motorlar Çalışmıyor

- Motor sürücü bağlantılarını kontrol edin
- 12V güç kaynağını kontrol edin
- Emergency button'un basılı olmadığından emin olun
- Arduino Serial Monitor'da `m 100 100` ile test edin

### ❌ TF Gözükmüyor

```bash
# TF tree kontrolü
ros2 run tf2_tools view_frames

# TF listesi
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## 📝 Önemli Parametreler

### Robot Ölçüleri (Teknofest-AGV)
- `wheel_separation`: 0.43 m
- `wheel_radius`: 0.050 m (controller), 0.055 m (URDF görünüm)
- `encoder_counts_per_rev`: 4000

### Baud Rate ve Port
- **Arduino**: 57600 baud
- **LIDAR**: 256000 baud
- **Arduino Port**: `/dev/ttyUSB0` veya `/dev/ttyACM0`
- **LIDAR Port**: `/dev/ttyUSB1` (genellikle)

### PID Parametreleri
- `pid_p`: 20
- `pid_d`: 12
- `pid_i`: 0
- `pid_o`: 50

---

## ✅ Kontrol Listesi

### Donanım
- [ ] Arduino Mega 2560 bağlı
- [ ] Motor sürücüleri (BTS7960) bağlı
- [ ] Encoder'lar bağlı ve çalışıyor
- [ ] LIDAR bağlı
- [ ] Raspberry Pi bağlı
- [ ] Güç kaynakları bağlı

### Yazılım
- [ ] Arduino firmware yüklendi
- [ ] ROS2 Jazzy kurulu
- [ ] Workspace build edildi
- [ ] Serial port izinleri ayarlandı
- [ ] libserial-dev kurulu

### Test
- [ ] Arduino Serial Monitor testi başarılı
- [ ] ROS2 Control bağlantısı başarılı
- [ ] Encoder değerleri okunuyor
- [ ] Motorlar çalışıyor
- [ ] LIDAR verisi geliyor
- [ ] TF tree doğru

---

## 🎯 Hızlı Başlangıç

```bash
# 1. Arduino firmware yükle (Arduino IDE'de)

# 2. Raspberry Pi'de workspace build et
cd ~/ros2_ws
colcon build

# 3. Source et
source install/setup.bash

# 4. Robot'u başlat
ros2 launch my_robot_bringup real_robot_exploration.launch.py \
    serial_port:=/dev/ttyUSB0 \
    lidar_port:=/dev/ttyUSB1
```

---

**Son Güncelleme**: 2025-01-16  
**ROS2 Versiyonu**: Jazzy  
**Robot**: Teknofest-AGV Uyumlu
