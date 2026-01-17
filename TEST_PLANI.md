# 🧪 Robot Test Planı - Raspberry Pi

Bu doküman, robotun tüm bileşenlerini adım adım test etmek için hazırlanmıştır.

## 📋 Test Öncesi Hazırlık

### 1. Sistem Kontrolü

```bash
# Raspberry Pi'de workspace'i source et
cd ~/ros2_ws
source install/setup.bash

# Serial port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# Beklenen çıktı:
# crw-rw---- 1 root dialout 188, 0 ... /dev/ttyUSB0  (Arduino)
# crw-rw---- 1 root dialout 188, 1 ... /dev/ttyUSB1  (LIDAR - varsa)
```

### 2. Port İzinleri Kontrolü

```bash
# Dialout grubunda mıyız?
groups $USER | grep dialout

# Değilse ekle (çıkış yapıp tekrar giriş yap)
sudo usermod -a -G dialout $USER
newgrp dialout
```

---

## 🔌 TEST 1: Arduino Serial Port Bağlantısı

**Amaç**: Arduino ile seri port haberleşmesinin çalıştığını doğrulamak

### Adımlar:

```bash
# 1. Miniterm kurulumu (yoksa)
sudo apt install miniterm

# 2. Arduino'ya bağlan (57600 baud)
miniterm /dev/ttyUSB0 57600

# 3. Arduino'ya komut gönder:
```

### Test Komutları (miniterm içinde):

```
z<Enter>     # Baud rate kontrolü -> "57600" yazmalı
e<Enter>     # Encoder okuma -> "0 0" veya encoder değerleri yazmalı
f<Enter>     # Batarya voltajı -> "12.50:95.0" gibi değer yazmalı
```

### ✅ Başarı Kriterleri:
- [ ] `z` komutu "57600" döndürüyor
- [ ] `e` komutu encoder değerleri döndürüyor (0 0 veya gerçek değerler)
- [ ] `f` komutu batarya değerleri döndürüyor
- [ ] Hata mesajı yok

### ❌ Sorun Varsa:
```bash
# Port kontrolü
ls -l /dev/ttyUSB0

# Başka bir program kullanıyor mu?
sudo lsof /dev/ttyUSB0

# İzinleri düzelt
sudo chmod 666 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0
```

**Çıkış**: `Ctrl+]` sonra `quit`

---

## 🔄 TEST 2: Encoder Testi

**Amaç**: Encoder'ların doğru çalıştığını ve değer okuduğunu doğrulamak

### Adımlar:

```bash
# 1. ROS2 Control başlat (sadece encoder okuma için)
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### Başka Terminalde:

```bash
# 2. Source et
source ~/ros2_ws/install/setup.bash

# 3. Joint states'i dinle (encoder değerleri)
ros2 topic echo /joint_states --once
```

### Beklenen Çıktı:
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- left_wheel_joint
- right_wheel_joint
position:  # Encoder pozisyonları (radyan)
- 0.0  # veya gerçek değer
- 0.0  # veya gerçek değer
velocity:  # Encoder hızları (rad/s)
- 0.0
- 0.0
```

### Manuel Test (Robot'u Elle Hareket Ettir):

```bash
# Robot'u elle hareket ettir ve encoder değerlerini izle
ros2 topic echo /joint_states
```

### ✅ Başarı Kriterleri:
- [ ] `/joint_states` topic'i yayınlanıyor
- [ ] `left_wheel_joint` ve `right_wheel_joint` görünüyor
- [ ] Robot hareket ettirildiğinde `position` değerleri değişiyor
- [ ] `velocity` değerleri hareket sırasında sıfırdan farklı

### ❌ Sorun Varsa:
```bash
# Encoder bağlantılarını kontrol et (Arduino pin 2,3,18,19)
# Miniterm ile test et:
miniterm /dev/ttyUSB0 57600
# e<Enter> -> encoder değerleri okunuyor mu?
```

---

## ⚙️ TEST 3: Motor Sürücü Testi (DİKKATLİ!)

**Amaç**: Motor sürücülerinin çalıştığını doğrulamak

### ⚠️ UYARI: Robot'u güvenli bir yerde test edin, motorlar çalışacak!

### Adımlar:

```bash
# 1. ROS2 Control başlat
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### Başka Terminalde:

```bash
# 2. Source et
source ~/ros2_ws/install/setup.bash

# 3. ÇOK YAVAŞ HAREKET TESTİ (0.05 m/s - çok yavaş!)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# 4. HEMEN DURDUR!
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Test Senaryoları:

#### A) İleri Hareket
```bash
# Çok yavaş ileri (0.05 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
sleep 2
# Durdur
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#### B) Geri Hareket
```bash
# Çok yavaş geri (-0.05 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: -0.05}, angular: {z: 0.0}}" --once
sleep 2
# Durdur
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#### C) Dönüş (Yerinde)
```bash
# Çok yavaş dönüş (0.1 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.1}}" --once
sleep 2
# Durdur
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Encoder Kontrolü (Motor Çalışırken):

```bash
# Başka terminalde encoder değerlerini izle
ros2 topic echo /joint_states
```

### ✅ Başarı Kriterleri:
- [ ] Motorlar komut alıyor ve hareket ediyor
- [ ] İleri komutunda robot ileri gidiyor
- [ ] Geri komutunda robot geri gidiyor
- [ ] Dönüş komutunda robot dönüyor
- [ ] Encoder değerleri hareket sırasında değişiyor
- [ ] Durdurma komutu motorları durduruyor

### ❌ Sorun Varsa:
```bash
# Motor sürücü bağlantılarını kontrol et
# BTS7960 enable pin'leri (R_EN, L_EN) 5V'a bağlı mı?
# PWM pin'leri doğru mu? (Sol: 9,6 | Sağ: 11,10)
# 12V güç kaynağı bağlı mı?

# Miniterm ile test:
miniterm /dev/ttyUSB0 57600
# m 10 10<Enter>  # Çok düşük değerlerle test
# m 0 0<Enter>    # Durdur
```

---

## 📊 TEST 4: Odometry Testi

**Amaç**: Odometry'nin doğru yayınlandığını doğrulamak

### Adımlar:

```bash
# 1. ROS2 Control başlat
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### Başka Terminalde:

```bash
# 2. Source et
source ~/ros2_ws/install/setup.bash

# 3. Odometry topic'ini dinle
ros2 topic echo /odom
```

### Beklenen Çıktı:
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
```

### Robot'u Hareket Ettir:

```bash
# Robot'u elle veya komutla hareket ettir
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Odometry değerlerini izle
ros2 topic echo /odom
```

### ✅ Başarı Kriterleri:
- [ ] `/odom` topic'i yayınlanıyor
- [ ] `frame_id: odom` ve `child_frame_id: base_link` doğru
- [ ] Robot hareket ettirildiğinde `pose.position` değişiyor
- [ ] Robot hareket ettirildiğinde `twist.linear.x` sıfırdan farklı
- [ ] Robot döndürüldüğünde `pose.orientation` değişiyor

### ❌ Sorun Varsa:
```bash
# TF kontrolü
ros2 run tf2_ros tf2_echo odom base_link

# Joint states kontrolü
ros2 topic echo /joint_states
```

---

## 🗺️ TEST 5: TF (Transform) Testi

**Amaç**: TF tree'nin doğru yayınlandığını doğrulamak

### Adımlar:

```bash
# 1. ROS2 Control başlat
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### Başka Terminalde:

```bash
# 2. Source et
source ~/ros2_ws/install/setup.bash

# 3. TF tree'yi görselleştir
ros2 run tf2_tools view_frames

# 4. TF listesini kontrol et
ros2 run tf2_ros tf2_echo odom base_link
```

### Beklenen TF Tree:
```
odom -> base_link (diff_drive_controller tarafından)
base_link -> base_footprint (URDF'de fixed joint)
base_link -> laser_link (URDF'de fixed joint)
```

### ✅ Başarı Kriterleri:
- [ ] `odom -> base_link` transform'u var
- [ ] `base_link -> base_footprint` transform'u var
- [ ] `base_link -> laser_link` transform'u var
- [ ] Transform değerleri mantıklı (pozisyon, oryantasyon)

### ❌ Sorun Varsa:
```bash
# TF listesi
ros2 run tf2_ros tf2_echo odom base_link

# TF tree görselleştirme
ros2 run tf2_tools view_frames
# frames.pdf dosyasını kontrol et
```

---

## 📡 TEST 6: LIDAR Testi

**Amaç**: LIDAR'ın çalıştığını ve veri yayınladığını doğrulamak

### Adımlar:

```bash
# 1. LIDAR node'unu başlat
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB1 \
    -p serial_baudrate:=256000 \
    -p frame_id:=laser_link
```

### Başka Terminalde:

```bash
# 2. Source et
source ~/ros2_ws/install/setup.bash

# 3. Scan topic'ini dinle
ros2 topic echo /scan --once
```

### Beklenen Çıktı:
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: laser_link
angle_min: -3.14159
angle_max: 3.14159
angle_increment: 0.0174533
time_increment: 0.0
scan_time: 0.1
range_min: 0.15
range_max: 12.0
ranges: [0.5, 0.6, 0.7, ...]  # Mesafe değerleri (metre)
intensities: [100, 120, 110, ...]  # Yoğunluk değerleri
```

### ✅ Başarı Kriterleri:
- [ ] `/scan` topic'i yayınlanıyor
- [ ] `frame_id: laser_link` doğru
- [ ] `ranges` dizisi dolu (engel mesafeleri)
- [ ] `angle_min` ve `angle_max` mantıklı (-π ile π arası)
- [ ] Robot çevresindeki engeller algılanıyor

### ❌ Sorun Varsa:
```bash
# LIDAR port kontrolü
ls -l /dev/ttyUSB1

# LIDAR bağlantısını kontrol et
# USB kablosu veri aktarımını destekliyor mu?
# LIDAR güç kaynağı bağlı mı?

# Farklı port dene
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=256000
```

---

## 🔄 TEST 7: Tam Sistem Testi (ROS2 Control + LIDAR)

**Amaç**: Tüm sistemin birlikte çalıştığını doğrulamak

### Adımlar:

```bash
# 1. Tam sistem başlat (ROS2 Control + LIDAR)
ros2 launch my_robot_bringup real_robot_exploration.launch.py \
    serial_port:=/dev/ttyUSB0 \
    lidar_port:=/dev/ttyUSB1 \
    use_rviz:=false
```

### Başka Terminallerde:

#### Terminal 2: Topic Kontrolü
```bash
source ~/ros2_ws/install/setup.bash

# Tüm topic'leri listele
ros2 topic list

# Beklenen topic'ler:
# /cmd_vel
# /joint_states
# /odom
# /scan
# /tf
# /tf_static
```

#### Terminal 3: Motor Testi
```bash
source ~/ros2_ws/install/setup.bash

# Çok yavaş hareket testi
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
sleep 3
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

#### Terminal 4: Sistem Durumu
```bash
source ~/ros2_ws/install/setup.bash

# Node listesi
ros2 node list

# Beklenen node'lar:
# /controller_manager
# /diff_drive_controller
# /joint_state_broadcaster
# /robot_state_publisher
# /rplidar_node
```

### ✅ Başarı Kriterleri:
- [ ] Tüm topic'ler yayınlanıyor
- [ ] Motorlar çalışıyor
- [ ] Encoder değerleri okunuyor
- [ ] Odometry yayınlanıyor
- [ ] LIDAR verisi geliyor
- [ ] TF tree doğru
- [ ] Hata mesajı yok

---

## 📋 Test Kontrol Listesi

### Donanım Testleri
- [ ] **TEST 1**: Arduino Serial Port Bağlantısı ✅
- [ ] **TEST 2**: Encoder Testi ✅
- [ ] **TEST 3**: Motor Sürücü Testi ✅
- [ ] **TEST 6**: LIDAR Testi ✅

### Yazılım Testleri
- [ ] **TEST 4**: Odometry Testi ✅
- [ ] **TEST 5**: TF (Transform) Testi ✅
- [ ] **TEST 7**: Tam Sistem Testi ✅

---

## 🐛 Yaygın Sorunlar ve Çözümleri

### 1. Serial Port Bulunamıyor
```bash
# Port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# İzinleri düzelt
sudo chmod 666 /dev/ttyUSB0
sudo chown root:dialout /dev/ttyUSB0

# Dialout grubuna ekle
sudo usermod -a -G dialout $USER
newgrp dialout
```

### 2. Encoder Değerleri 0
```bash
# Encoder bağlantılarını kontrol et
# Arduino pin 2,3 (sol) ve 18,19 (sağ)
# Miniterm ile test:
miniterm /dev/ttyUSB0 57600
# e<Enter> -> encoder değerleri okunuyor mu?
```

### 3. Motorlar Çalışmıyor
```bash
# Motor sürücü bağlantılarını kontrol et
# BTS7960 enable pin'leri (R_EN, L_EN) 5V'a bağlı mı?
# PWM pin'leri doğru mu? (Sol: 9,6 | Sağ: 11,10)
# 12V güç kaynağı bağlı mı?
# Emergency button basılı değil mi?
```

### 4. TF Gözükmüyor
```bash
# TF tree kontrolü
ros2 run tf2_tools view_frames

# TF listesi
ros2 run tf2_ros tf2_echo odom base_link

# Robot state publisher çalışıyor mu?
ros2 node list | grep robot_state_publisher
```

### 5. LIDAR Verisi Gelmiyor
```bash
# LIDAR port kontrolü
ls -l /dev/ttyUSB1

# LIDAR node çalışıyor mu?
ros2 node list | grep rplidar

# Farklı port dene
ros2 run rplidar_ros rplidar_node --ros-args \
    -p serial_port:=/dev/ttyUSB0
```

---

## 🎯 Hızlı Test Komutları

```bash
# Tüm testleri hızlıca çalıştır
source ~/ros2_ws/install/setup.bash

# 1. Serial port kontrolü
ls -l /dev/ttyUSB* /dev/ttyACM*

# 2. ROS2 Control başlat
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0

# 3. Başka terminalde test
ros2 topic list                    # Topic'leri listele
ros2 topic echo /joint_states      # Encoder değerleri
ros2 topic echo /odom              # Odometry
ros2 run tf2_ros tf2_echo odom base_link  # TF

# 4. Motor testi (dikkatli!)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05}, angular: {z: 0.0}}" --once
```

---

**Son Güncelleme**: 2025-01-16  
**Test Ortamı**: Raspberry Pi 5 + ROS2 Jazzy
