# my_robot_bringup

Bu paket, robot sistemini başlatmak için gerekli tüm launch dosyalarını ve konfigürasyon dosyalarını içerir.

## 📁 Klasör Yapısı

```
my_robot_bringup/
├── launch/                    # Launch dosyaları (ROS2 sistemini başlatır)
│   ├── arduino/               # Arduino ile ilgili launch dosyaları
│   ├── autonomous_exploration.launch.py  # Otonom keşif (Gazebo simülasyonu)
│   ├── real_robot_bringup.launch.py      # Gerçek robot temel bringup
│   └── real_robot_exploration.launch.py  # Gerçek robot full system
├── config/                    # Konfigürasyon dosyaları
│   ├── explore_params.yaml    # Frontier explorer parametreleri
│   ├── nav2_params.yaml       # Nav2 navigasyon parametreleri
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox parametreleri
│   ├── laser_filter.yaml      # LIDAR box filter konfigürasyonu
│   └── my_robot_controllers.yaml  # ROS2 Control controller config
├── arduino/                   # Arduino ile ilgili dokümantasyon
│   ├── firmware/              # Arduino firmware kodları
│   └── ARDUINO_SERIAL_PROTOCOL.md  # Serial protokol dokümantasyonu
├── rviz/                      # RViz görselleştirme config'leri
└── maps/                      # Harita dosyaları (.pgm, .yaml)
```

## 🚀 Launch Dosyaları

### Simülasyon
- `autonomous_exploration.launch.py` - Gazebo'da otonom keşif

### Gerçek Robot
- `real_robot_bringup.launch.py` - Temel bringup (Arduino bridge, TF)
- `real_robot_exploration.launch.py` - Full system (bringup + LIDAR + SLAM + Nav2 + exploration)

### Arduino
- `arduino/arduino_robot_bringup.launch.py` - Arduino bridge bringup

## ⚙️ Config Dosyaları

- `explore_params.yaml` - Frontier explorer ayarları (goal distance, cooldown, vb.)
- `nav2_params.yaml` - Nav2 stack ayarları (controller, planner, costmaps, vb.)
- `mapper_params_online_async.yaml` - SLAM Toolbox ayarları (loop closure, correlation, vb.)
- `laser_filter.yaml` - LIDAR box filter (robot gövdesini filtreler)
- `my_robot_controllers.yaml` - ROS2 Control controller ayarları

## 📚 Detaylı Bilgi

- Arduino protokol: `arduino/ARDUINO_SERIAL_PROTOCOL.md`
- Arduino firmware: `arduino/firmware/README.md`
