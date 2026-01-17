# Raspberry Pi Terminal Komutları ve RViz Ayarları

## 🚀 Raspberry Pi Terminalinde Yapılacaklar

### 1. Workspace'e Git ve Source Et
```bash
cd ~/ros2_ws
source install/setup.bash
```

### 2. Serial Port Kontrolü
```bash
# Arduino'nun hangi portta olduğunu kontrol et
ls -l /dev/ttyUSB* /dev/ttyACM*

# Genellikle:
# - Arduino: /dev/ttyUSB0
# - LIDAR: /dev/ttyUSB1
```

### 3. Launch Dosyasını Çalıştır

#### Seçenek A: Tam Sistem (SLAM + Nav2 + Exploration)
```bash
ros2 launch my_robot_bringup real_robot_exploration.launch.py serial_port:=/dev/ttyUSB0 use_rviz:=false
```

#### Seçenek B: Sadece TF'leri Test Et (ROS2 Control)
```bash
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### 4. TF Kontrolü (Başka bir terminalde)
```bash
# TF tree'yi görüntüle
ros2 run tf2_tools view_frames

# Belirli bir transform'u kontrol et
ros2 run tf2_ros tf2_echo odom base_link

# Tüm TF'leri listele
ros2 topic echo /tf --once
```

### 5. Topic Kontrolü
```bash
# Yayınlanan topic'leri listele
ros2 topic list

# Odometry kontrolü
ros2 topic echo /odom --once

# LIDAR kontrolü
ros2 topic echo /scan --once

# Motor komutu gönder (test için)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 📊 RViz2'de Eklenmesi Gereken Display'ler

### RViz2'yi Başlat
```bash
# Eğer launch dosyasında use_rviz:=true yaptıysanız otomatik açılır
# Veya manuel olarak:
rviz2
```

### RViz2'de Eklenmesi Gereken Display'ler (sırayla):

#### 1. **TF (Transform)**
- **Add → By display type → TF**
- **Description**: Tüm frame'leri ve transform'ları gösterir
- **Frame**: `odom` (veya `map` SLAM başladıktan sonra)
- **Show Names**: ✅ (frame isimlerini gösterir)
- **Show Axes**: ✅ (koordinat eksenlerini gösterir)
- **Show Arrows**: ✅ (transform yönlerini gösterir)

#### 2. **RobotModel**
- **Add → By display type → RobotModel**
- **Description**: URDF'den robot modelini gösterir
- **Robot Description**: `robot_description`
- **TF Prefix**: (boş bırakın)

#### 3. **LaserScan** (LIDAR)
- **Add → By topic → `/scan_filtered`** (veya `/scan`)
- **Description**: LIDAR tarama verilerini gösterir
- **Topic**: `/scan_filtered` (box filter uygulanmış) veya `/scan`
- **Size (m)**: `0.05` (nokta boyutu)
- **Color**: `Intensity` veya `Flat`
- **Decay Time (s)**: `0.0` (tüm noktaları göster)

#### 4. **Map** (SLAM başladıktan sonra)
- **Add → By topic → `/map`**
- **Description**: SLAM Toolbox'ın oluşturduğu haritayı gösterir
- **Topic**: `/map`
- **Color Scheme**: `costmap` (mavi: bilinmeyen, beyaz: serbest, siyah: engel)
- **Alpha**: `0.7` (şeffaflık)

#### 5. **Path** (Nav2 başladıktan sonra)
- **Add → By topic → `/plan`** (planner path)
- **Description**: Nav2'nin planladığı yolu gösterir
- **Topic**: `/plan`
- **Color**: `255; 0; 0` (kırmızı)
- **Line Style**: `Lines`
- **Line Width**: `0.03`

#### 6. **Path** (Controller path)
- **Add → By topic → `/local_plan`**
- **Description**: Controller'ın lokal planını gösterir
- **Topic**: `/local_plan`
- **Color**: `0; 255; 0` (yeşil)
- **Line Style**: `Lines`
- **Line Width**: `0.03`

#### 7. **PoseArray** (Frontier Explorer)
- **Add → By topic → `/frontiers`**
- **Description**: Frontier Explorer'ın bulduğu frontier noktalarını gösterir
- **Topic**: `/frontiers`
- **Color**: `255; 255; 0` (sarı)
- **Shape**: `Arrow (Flat)`
- **Length**: `0.3`

#### 8. **Marker** (Frontier Explorer)
- **Add → By topic → `/explore/frontiers`**
- **Description**: Frontier Explorer'ın görselleştirme marker'ları
- **Topic**: `/explore/frontiers`
- **Queue Size**: `10`

#### 9. **Pose** (Robot Pose)
- **Add → By topic → `/amcl_pose`** (eğer AMCL kullanıyorsanız)
- **Description**: Robot'un tahmin edilen pozisyonu
- **Topic**: `/amcl_pose`
- **Color**: `0; 0; 255` (mavi)
- **Shape**: `Arrow (Flat)`
- **Length**: `0.5`

#### 10. **Odometry** (Opsiyonel)
- **Add → By topic → `/odom`**
- **Description**: Odometry verilerini gösterir
- **Topic**: `/odom`
- **Shape**: `Arrow (Flat)`
- **Length**: `0.3`
- **Color**: `255; 0; 255` (magenta)

---

## 🎯 RViz2 Ayarları (Global Options)

### Fixed Frame
- **Fixed Frame**: `odom` (başlangıçta) → `map` (SLAM başladıktan sonra)
- **Background Color**: `48; 48; 48` (koyu gri)

### View Settings
- **Type**: `Orbit` (robot etrafında dönebilirsiniz)
- **Focal Point**: Robot'un merkezi
- **Distance**: `5.0` (uzaklık)
- **Field of View**: `40.0`

---

## 🔧 Sorun Giderme

### TF Gözükmüyorsa:
```bash
# TF'leri kontrol et
ros2 run tf2_ros tf2_echo odom base_link

# Eğer hata veriyorsa, launch dosyasını kontrol et:
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
```

### LIDAR Gözükmüyorsa:
```bash
# LIDAR topic'ini kontrol et
ros2 topic echo /scan --once

# LIDAR portunu kontrol et (genellikle /dev/ttyUSB1)
ls -l /dev/ttyUSB*
```

### Map Gözükmüyorsa:
```bash
# Map topic'ini kontrol et
ros2 topic echo /map --once

# SLAM Toolbox'ın çalıştığını kontrol et
ros2 node list | grep slam
```

---

## 📝 Notlar

1. **RViz2 X11 Forwarding**: SSH ile bağlanıyorsanız, X11 forwarding kullanın:
   ```bash
   ssh -X raspi@<raspberry_pi_ip>
   ```

2. **VNC Kullanımı**: Alternatif olarak VNC server kurup kullanabilirsiniz.

3. **Frame Sırası**: 
   - `map` → `odom` → `base_link` → `base_footprint` → `laser_link`

4. **Topic İsimleri**:
   - Odometry: `/odom`
   - LIDAR: `/scan` (ham) veya `/scan_filtered` (filtrelenmiş)
   - Map: `/map`
   - Cmd Vel: `/cmd_vel`
