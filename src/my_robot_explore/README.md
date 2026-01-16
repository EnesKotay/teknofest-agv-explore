# my_robot_explore

Otonom keşif ve odometry yönetimi için Python node'ları içerir.

## 📁 Dosya Yapısı

```
my_robot_explore/
├── my_robot_explore/          # Python modülü
│   ├── frontier_explorer.py   # Frontier-based otonom keşif algoritması
│   ├── arduino_bridge.py      # Arduino serial bridge (cmd_vel -> motor, encoder -> odom)
│   └── odom_tf_broadcaster.py # Odometry -> TF transform yayınlar
├── launch/
│   └── explore.launch.py      # Frontier explorer launch dosyası
└── setup.py                   # Python paket tanımı
```

## 🔧 Node'lar

### 1. frontier_explorer.py

**Ne yapar**: Otonom keşif algoritması - haritadaki keşfedilmemiş alanları bulur ve robotu oraya gönderir.

**Özellikler**:
- Information Gain tabanlı hedef seçimi
- Frontier clustering (yakın frontierleri gruplar)
- Stuck detection (takılma algılama)
- Return-to-start (harita tamamlanınca başlangıca dön)
- Visualization (frontier'leri görselleştirir)

**Topic'ler**:
- Subscribes: `/map` (OccupancyGrid)
- Publishes: `/goal_pose` (PoseStamped), `/frontier_markers` (MarkerArray)

### 2. arduino_bridge.py

**Ne yapar**: Arduino ile seri port üzerinden haberleşir - ROS2 topic'leri ile Arduino komutları arasında köprü görevi görür.

**Özellikler**:
- `/cmd_vel` → Arduino motor komutları
- Arduino encoder → `/odom` (Odometry)
- Auto-stop (2 saniye komut gelmezse motorları durdurur)

**Parametreler**: Serial port, baud rate, wheel separation, wheel radius, encoder counts per rev

**Topic'ler**:
- Subscribes: `/cmd_vel` (Twist)
- Publishes: `/odom` (Odometry)

### 3. odom_tf_broadcaster.py

**Ne yapar**: `/odom` topic'inden TF transform yayınlar (odom → base_footprint).

**Neden gerekli**: SLAM ve Nav2 için TF tree'de odom frame'inin olması gerekir.

**Topic'ler**:
- Subscribes: `/odom` (Odometry)
- Publishes: TF (`odom` → `base_footprint`)

## 📊 Parametreler

Frontier explorer parametreleri: `../my_robot_bringup/config/explore_params.yaml`

## 🎯 Kullanım

```bash
# Frontier explorer'ı başlat
ros2 launch my_robot_explore explore.launch.py

# Arduino bridge'i başlat (launch dosyasından otomatik başlar)
```
