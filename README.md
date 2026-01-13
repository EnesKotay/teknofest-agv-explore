# Teknofest AGV - Otonom Keşif Robotu

ROS2 Jazzy tabanlı otonom keşif sistemi. Frontier-based exploration algoritması ile harita oluşturma ve otonom navigasyon.

## 🚀 Özellikler

- **Otonom Keşif:** Frontier-based exploration algoritması
- **SLAM:** SLAM Toolbox ile gerçek zamanlı harita oluşturma
- **Navigasyon:** Nav2 stack ile otonom navigasyon
- **Return-to-Start:** Keşif tamamlandıktan sonra başlangıç noktasına dönme
- **Gerçek Robot Desteği:** STM32 Nucleo + Micro-ROS entegrasyonu
- **LIDAR Desteği:** RPLIDAR A2 entegrasyonu

## 📋 Gereksinimler

- **ROS2:** Jazzy Jalisco (Ubuntu 24.04)
- **Python:** 3.10+
- **Donanım (Simülasyon):** Gazebo Gz
- **Donanım (Gerçek Robot):**
  - Raspberry Pi 5 (Ubuntu 24.04 Server)
  - STM32 Nucleo (F401RE veya F446RE)
  - RPLIDAR A2
  - Differential drive robot

## 📦 Kurulum

### 1. Workspace Oluşturma

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
```

### 2. Repository'yi Klonlama

```bash
git clone <repository-url> .
# veya
git clone <repository-url> my_robot_explore
cd my_robot_explore
```

### 3. Bağımlılıkları Kurma

```bash
cd ~/robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
```

### 4. Build

```bash
colcon build
source install/setup.bash
```

## 🎮 Kullanım

### Simülasyon (Gazebo)

```bash
# Otonom keşif (Gazebo simülasyonu)
ros2 launch my_robot_bringup autonomous_exploration.launch.py
```

### Gerçek Robot

```bash
# Temel bringup (STM32 + LIDAR)
ros2 launch my_robot_bringup real_robot_bringup.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=460800 \
    lidar_port:=/dev/ttyUSB0

# Full system (Otonom keşif)
ros2 launch my_robot_bringup real_robot_exploration.launch.py \
    serial_port:=/dev/ttyACM0 \
    baud_rate:=460800 \
    lidar_port:=/dev/ttyUSB0
```

## 📁 Proje Yapısı

```
my_robot_explore/
├── src/
│   ├── my_robot_bringup/          # Launch dosyaları ve config
│   ├── my_robot_description/      # URDF robot tanımı
│   └── my_robot_explore/          # Frontier explorer node
├── docs/                          # Dokümantasyon
└── README.md
```


## 🔧 Konfigürasyon

### Exploration Parametreleri

`src/my_robot_bringup/config/explore_params.yaml` dosyasından keşif parametrelerini ayarlayabilirsiniz:

- `min_goal_distance`: Minimum hedef mesafesi
- `goal_timeout_sec`: Hedef zaman aşımı
- `return_to_start`: Başlangıca dönme özelliği
- `roi_radius`: Keşif alanı yarıçapı

### Nav2 Parametreleri

`src/my_robot_bringup/config/nav2_params.yaml` dosyasından navigasyon parametrelerini ayarlayabilirsiniz.

## 🧪 Test

```bash
# Topic'leri kontrol et
ros2 topic list

# Odometry kontrolü
ros2 topic echo /odom

# Frontier'leri görselleştir
ros2 topic echo /frontier_markers

# TF tree
ros2 run tf2_tools view_frames
```

## 🤝 Katkıda Bulunma

1. Fork yapın
2. Feature branch oluşturun (`git checkout -b feature/AmazingFeature`)
3. Commit yapın (`git commit -m 'Add some AmazingFeature'`)
4. Push yapın (`git push origin feature/AmazingFeature`)
5. Pull Request açın

## 📝 Lisans

Bu proje Teknofest yarışması için geliştirilmiştir.

## 👥 Ekip

- **Sistem Koordinatörü:** Sistem entegrasyonu ve test
- **STM32 Geliştirici:** STM32 firmware ve Micro-ROS
- **LIDAR Geliştirici:** LIDAR entegrasyonu

## 🙏 Teşekkürler

- ROS2 topluluğu
- SLAM Toolbox geliştiricileri
- Nav2 geliştiricileri
- Micro-ROS topluluğu
