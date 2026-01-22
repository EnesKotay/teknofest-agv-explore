# Gerçek Robot Özellikleri (Teknofest AGV)

Bu dokümantasyon gerçek robotun (`/home/enes/Desktop/teknofest-agv-main (2)`) tüm fiziksel ve yazılımsal özelliklerini içermektedir.

## 📐 Fiziksel Özellikler

### Robot Gövdesi (Base)
- **Genişlik (base_width)**: 0.55 m
- **Uzunluk (base_length)**: 0.60 m
- **Yükseklik (base_height)**: 0.24 m
- **Aks Ofseti (axle_offset)**: 0.10 m
- **Robot Yarıçapı**: ~0.22 m (Nav2 costmap için)

### Tekerlekler
- **Tekerlek Yarıçapı (wheel_radius)**: 0.055 m (URDF) / 0.050 m (Controller)
- **Tekerlek Genişliği (wheel_len)**: 0.030 m
- **Tekerlek Ayrımı (wheel_separation)**: 0.43 m
- **Caster Tekerlek Yarıçapı**: 0.055 m
- **Tekerlek Damping**: 0.2

### LIDAR Sensörü
- **Model**: RPLIDAR A2M12
- **Seri Port**: `/dev/ttyUSB0`
- **Baudrate**: 115200
- **Tarama Frekansı**: 10.0 Hz
- **Açı Aralığı**: 0.0 - 2π rad (360°)
- **Minimum Menzil**: 0.15 m
- **Maksimum Menzil**: 12.0 m
- **LIDAR Yarıçapı**: 0.05 m
- **LIDAR Uzunluğu**: 0.045 m
- **LIDAR Yüksekliği**: base_height * 1.1 ≈ 0.264 m
- **Frame ID**: `lidar_link`
- **Topic**: `/scan`

### IMU Sensörü
- **Model**: MPU6050
- **Topic**: `/imu/mpu6050`
- **Gyro Full Scale**: ±250 deg/s (FS_SEL: 0)
- **Accelerometer Full Scale**: ±2 g (AFS_SEL: 0)
- **DLPF Config**: 0 (260 Hz bandwidth)
- **Clock Source**: Internal 8MHz oscillator
- **Gyro Offsets**:
  - X: -1.64212 deg/s
  - Y: -0.976061 deg/s
  - Z: -0.841023 deg/s
- **Accelerometer Offsets**:
  - X: -0.52414 m/s²
  - Y: 0.306007 m/s²
  - Z: 9.74877 m/s²

## ⚙️ Motor ve Kontrol Özellikleri

### Arduino Hardware Interface
- **Plugin**: `diffdrive_arduino/DiffDriveArduinoHardware`
- **Seri Port**: `/dev/ttyACM0`
- **Baudrate**: 57600
- **Loop Rate**: 40 Hz
- **Timeout**: 1000 ms
- **Firmware**: ROSArduinoBridge protokolü

### Encoder Özellikleri
- **Sol Encoder Counts/Rev**: 4000
- **Sağ Encoder Counts/Rev**: 4000
- **Encoder Çözünürlüğü**: Yüksek hassasiyet (4000 pulse/tur)

### PID Kontrol Parametreleri
- **P (Proportional)**: 20
- **I (Integral)**: 0
- **D (Derivative)**: 12
- **O (Output Limit)**: 50
- **Reverse Speed Threshold**: -0.1 m/s

### Diff Drive Controller
- **Controller Type**: `diff_drive_controller/DiffDriveController`
- **Update Rate**: 10 Hz
- **Publish Rate**: 50.0 Hz
- **Open Loop**: false (closed-loop control)
- **Odometry TF**: Enabled
- **Base Frame**: `base_link`
- **Odometry Frame**: `odom`
- **Cmd Vel Timeout**: 0.5 s

### Hız ve İvme Limitleri
- **Maksimum Doğrusal Hız**: ±1.0 m/s
- **Maksimum Açısal Hız**: ±1.0 rad/s
- **Maksimum Doğrusal İvme**: ±1.0 m/s²
- **Maksimum Açısal İvme**: ±1.0 rad/s²
- **Jerk Limitleri**: Yok (0.0)

### Odometry Kovaryans Matrisleri
- **Pose Kovaryans**: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
- **Twist Kovaryans**: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

## 🧭 Sensor Fusion (EKF)

### Extended Kalman Filter
- **Frequency**: 30.0 Hz
- **Mode**: 2D (two_d_mode: true)
- **Frames**:
  - Map Frame: `map`
  - Odometry Frame: `odom`
  - Base Frame: `base_link`
  - World Frame: `odom`

### EKF Sensor Konfigürasyonu

#### IMU (imu0)
- **Topic**: `/imu/mpu6050`
- **Kullanılan Veriler**:
  - Angular Velocity Z: ✅
  - Linear Acceleration X: ✅
- **Differential**: false

#### Odometry (odom0)
- **Topic**: `/diffbot_base_controller/odom`
- **Kullanılan Veriler**:
  - Position X, Y: ✅
  - Angular Velocity Z: ✅
- **Differential**: false

## 📡 ROS2 Topic'leri ve Frame'leri

### Frame Hierarchy
```
map → odom → base_link → base_footprint
                    ↓
                 laser (lidar_link)
```

### Önemli Topic'ler
- **LIDAR Scan**: `/scan` (LaserScan)
- **Odometry**: `/diffbot_base_controller/odom` (Odometry)
- **IMU**: `/imu/mpu6050` (Imu)
- **Cmd Vel**: `/cmd_vel` (Twist)
- **Joint States**: `/joint_states` (JointState)

### Joint'ler
- `left_wheel_joint` - Sol tekerlek
- `right_wheel_joint` - Sağ tekerlek

## 🎯 Nav2 Optimizasyon Parametreleri

### Robot Yarıçapı
- **Costmap Robot Radius**: 0.22 m
- **Inflation Radius**: 0.85 m
- **Cost Scaling Factor**: 4.0

### Goal Checker Toleransları
- **XY Goal Tolerance**: 0.5 m
- **Yaw Goal Tolerance**: 0.785 rad (~45°)

### Controller Parametreleri (Regulated Pure Pursuit)
- **Desired Linear Velocity**: 0.35 m/s
- **Lookahead Distance**: 0.65 m
- **Min Lookahead**: 0.3 m
- **Max Lookahead**: 0.9 m
- **Rotate to Heading Angular Velocity**: 1.2 rad/s
- **Rotate to Heading Min Angle**: 0.5 rad (~29°)

## 🔧 Frontier Explorer için Önerilen Parametreler

Bu robot özelliklerine göre optimize edilmiş parametreler:

```yaml
frontier_explorer:
  ros__parameters:
    # Robot özelliklerine göre optimize edilmiş parametreler
    min_goal_distance: 1.2          # Robot genişliği (0.55m) * 2 + margin
    goal_cooldown_sec: 1.0
    goal_timeout_sec: 60.0
    return_to_start_timeout_sec: 120.0
    
    # Robot yarıçapına göre güvenlik
    safety_cell_radius: 4           # Robot radius (0.22m) / resolution (0.05m) ≈ 4.4 → 4
    
    # Robot hız limitlerine göre
    stuck_velocity_threshold: 0.05  # Max velocity (1.0 m/s) * 0.05
    stuck_distance_threshold: 0.5   # Robot uzunluğu (0.6m) civarı
    
    # ROI radius - LIDAR menziline göre
    roi_radius: 15.0                # LIDAR max range (12m) * 1.25
    roi_radius_min: 10.0
    roi_radius_max: 20.0
    
    # Start position tolerance - Nav2 goal checker'a göre
    start_position_tolerance: 0.6  # Nav2 xy_goal_tolerance (0.5m) + margin
```

## 📝 Notlar

1. **Tekerlek Yarıçapı Farkı**: URDF'te 0.055m, Controller'da 0.050m tanımlı. Controller değeri kullanılmalı (odometry için).

2. **Encoder Çözünürlüğü**: 4000 counts/rev yüksek hassasiyet sağlar, ancak PID tuning önemlidir.

3. **LIDAR Menzili**: 12m maksimum menzil, ancak güvenilir menzil ~8-10m olabilir.

4. **IMU Kalibrasyonu**: Offset değerleri robotun montajına göre değişebilir, kalibrasyon gerekebilir.

5. **Seri Port**: Arduino `/dev/ttyACM0`, LIDAR `/dev/ttyUSB0` - sistemde farklı olabilir.

6. **Robot Yarıçapı**: Nav2 costmap için 0.22m kullanılıyor (robot genişliği/2 + margin).

7. **Hız Limitleri**: Maksimum 1.0 m/s, ancak güvenli operasyon için 0.35-0.5 m/s önerilir.

## 🔗 İlgili Dosyalar

- **URDF**: `diffdrive_arduino/description/urdf/diffbot_description.urdf.xacro`
- **ROS2 Control**: `diffdrive_arduino/description/ros2_control/diffbot.ros2_control.xacro`
- **Controller Config**: `diffdrive_arduino/bringup/config/diffbot_controllers.yaml`
- **EKF Config**: `diffdrive_arduino/bringup/config/ekf.yaml`
- **LIDAR Config**: `diffdrive_arduino/config/lidar_params.yaml`
- **IMU Config**: `imu/config/params.yaml`
