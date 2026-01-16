# my_robot_description

Robot'un fiziksel modelini tanımlayan URDF dosyalarını ve Gazebo simülasyon dosyalarını içerir.

## 📁 Klasör Yapısı

```
my_robot_description/
├── urdf/                      # URDF/Xacro robot tanım dosyaları
│   ├── my_robot.urdf.xacro   # Ana robot URDF (base, wheels, LIDAR)
│   └── my_robot.materials.xacro  # Görsel materyal tanımları
├── ros2_control/              # ROS2 Control konfigürasyonu
│   └── my_robot.ros2_control.xacro  # Hardware interface tanımı
├── models/                    # Gazebo model dosyaları
│   └── my_robot/
│       ├── model.sdf         # Gazebo SDF model
│       └── model.config      # Gazebo model config
└── worlds/                    # Gazebo dünya dosyaları
    ├── empty_slam_world.sdf   # Boş dünya (SLAM test için)
    └── example_walls_world.sdf # Örnek duvarlı dünya
```

## 🤖 Robot Özellikleri

### Fiziksel Boyutlar
- **Base (Gövde)**: 0.60m x 0.55m x 0.24m
- **Tekerlek yarıçapı**: 0.055m
- **Tekerlekler arası mesafe**: 0.30m (wheel_separation)
- **LIDAR pozisyonu**: Base üstünde, ön kısımda

### Joint'ler
- `left_wheel_joint` - Sol tekerlek (continuous)
- `right_wheel_joint` - Sağ tekerlek (continuous)
- `caster_frontal_wheel_joint` - Ön caster tekerlek (fixed)

### Frame'ler
- `base_footprint` - Root frame (z=0, yer seviyesi)
- `base_link` - Robot gövdesi
- `laser_link` - LIDAR frame'i

## 🔧 URDF Dosyaları

### my_robot.urdf.xacro

Ana robot tanım dosyası. Şunları içerir:
- Robot boyutları (property'ler olarak)
- Base link (collision, visual, inertial)
- Tekerlekler (left, right, caster)
- LIDAR sensörü

**Not**: Gazebo için `model.sdf` kullanılır, gerçek robot için URDF kullanılır.

## 🌍 Gazebo Worlds

- `empty_slam_world.sdf` - Boş dünya, SLAM testleri için
- `example_walls_world.sdf` - Örnek duvarlı dünya, navigation testleri için

## 📐 ROS2 Control

`ros2_control/my_robot.ros2_control.xacro` - ROS2 Control hardware interface tanımı:
- Hardware plugin: `my_robot_hardware/MyRobotSystem`
- Joint interface'leri: velocity command, position/velocity state

## 🎮 Kullanım

```bash
# Robot'u görselleştir (RViz)
ros2 launch my_robot_description display.launch.py

# Gazebo simülasyonu
ros2 launch my_robot_bringup autonomous_exploration.launch.py
```
