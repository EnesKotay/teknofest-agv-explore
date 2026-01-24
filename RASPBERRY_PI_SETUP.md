# Raspberry Pi 5 Kurulum Rehberi

Bu rehber, Raspberry Pi 5'te projeyi çalıştırmak için gerekli tüm adımları içerir.

## 📋 Ön Gereksinimler

- Raspberry Pi 5 (Ubuntu 24.04 Server veya Desktop)
- ROS2 Jazzy kurulu
- STM32 USB-UART modülü bağlı
- LIDAR bağlı (opsiyonel)

## 🔧 Kurulum Adımları

### 1. Sistem Güncellemesi

```bash
sudo apt update
sudo apt upgrade -y
```

### 2. ROS2 Jazzy Kurulumu (Eğer yoksa)

```bash
# Ubuntu 24.04 için ROS2 Jazzy
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 3. Gerekli Paketler

```bash
# ROS2 paketleri
sudo apt install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-navigator \
    ros-jazzy-tf2-ros \
    python3-pip

# Python bağımlılıkları
pip3 install pyserial

# Veya sistem paketi
sudo apt install python3-serial
```

### 4. Serial Port Permission

```bash
# Kullanıcıyı dialout grubuna ekle
sudo usermod -aG dialout $USER

# Yeni grup aktif olması için logout/login yap
# Veya:
newgrp dialout

# Kontrol et
groups | grep dialout
```

### 5. Projeyi Klonlama

```bash
# Workspace oluştur
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Repository'yi klonla
git clone <repository-url> .

# Veya mevcut workspace'e ekle
cd ~/teknofest-agv-explore
```

### 6. Bağımlılıkları Kurma

```bash
cd ~/teknofest-agv-explore  # veya ~/robot_ws

# rosdep kurulumu (eğer yoksa)
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

# Bağımlılıkları kur
rosdep install --from-paths src --ignore-src -y
```

### 7. Build

```bash
cd ~/teknofest-agv-explore  # veya ~/robot_ws

# Build et
colcon build

# Workspace'i source et
source install/setup.bash

# Kalıcı olması için .bashrc'ye ekle
echo "source ~/teknofest-agv-explore/install/setup.bash" >> ~/.bashrc
```

### 8. Serial Port Kontrolü

```bash
# Serial port'ları listele
ls -l /dev/ttyUSB* /dev/ttyACM*

# STM32 USB-UART modülü genellikle /dev/ttyUSB0
# LIDAR genellikle /dev/ttyUSB1

# Yeni cihaz eklendi mi kontrol et
dmesg | tail
```

## 🚀 Çalıştırma

### Temel Test (Serial Bridge)

```bash
# Workspace'i source et
source ~/teknofest-agv-explore/install/setup.bash

# Serial bridge'i test et
ros2 run my_robot_explore stm32_serial_bridge \
    --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p baud_rate:=115200

# Başka terminal'de encoder verilerini gör
ros2 topic echo /joint_states
```

### Temel Bringup (ROS2 Control)

```bash
ros2 launch my_robot_bringup real_robot_control_bringup.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200
```

### Otonom SLAM

```bash
ros2 launch my_robot_bringup real_robot_control_slam.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    lidar_port:=/dev/ttyUSB1
```

## 🔍 Sorun Giderme

### Serial Port Görünmüyor

```bash
# USB cihazlarını kontrol et
lsusb

# Yeni cihaz eklendi mi?
dmesg | tail

# Manuel olarak port'u kontrol et
sudo chmod 666 /dev/ttyUSB0
```

### Permission Denied

```bash
# dialout grubuna ekle
sudo usermod -aG dialout $USER
newgrp dialout

# Kontrol et
groups | grep dialout
```

### pyserial Bulunamadı

```bash
# pip ile kur
pip3 install pyserial

# Veya sistem paketi
sudo apt install python3-serial
```

### Build Hatası

```bash
# Eksik paketleri kontrol et
rosdep install --from-paths src --ignore-src -y

# Temiz build
rm -rf build install log
colcon build
```

## 📝 Notlar

- Serial port genellikle `/dev/ttyUSB0` (STM32) ve `/dev/ttyUSB1` (LIDAR)
- Baud rate: 115200 (STM32 için)
- STM32 firmware çalışıyor olmalı
- Motor sürücülere güç bağlı olmalı
