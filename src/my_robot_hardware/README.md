# my_robot_hardware

ROS2 Control hardware interface - Arduino ile seri port haberleşmesi için C++ implementasyonu.

## 📁 Dosya Yapısı

```
my_robot_hardware/
├── hardware/
│   ├── my_robot_system.cpp              # Ana hardware interface implementation
│   └── include/my_robot_hardware/
│       ├── my_robot_system.hpp          # Hardware interface class header
│       └── arduino_comms.hpp            # Serial iletişim sınıfı (LibSerial)
├── include/my_robot_hardware/
│   └── wheel.hpp                        # Tekerlek sınıfı (encoder, position, velocity)
├── CMakeLists.txt                       # C++ build konfigürasyonu
├── package.xml                          # ROS2 paket tanımı
├── my_robot_hardware.xml                # Plugin tanımı (ROS2 Control)
└── README.md                            # Bu dosya
```

## 🔧 Ana Dosyalar

### my_robot_system.cpp / .hpp

**Ne yapar**: ROS2 Control SystemInterface implementasyonu - Arduino'dan encoder okur, motor komutları gönderir.

**Lifecycle Metodları**:
- `on_init()` - Parametreleri okur, interface'leri doğrular
- `on_configure()` - Serial port'a bağlanır
- `on_activate()` - PID parametrelerini gönderir
- `read()` - Encoder okur, position/velocity hesaplar
- `write()` - Velocity komutlarını motor komutlarına çevirir

### arduino_comms.hpp

**Ne yapar**: Arduino ile seri port haberleşmesi yönetir (LibSerial kütüphanesi kullanır).

**Fonksiyonlar**:
- `connect()` / `disconnect()` - Port bağlantısı
- `read_encoder_values()` - `"e\r"` komutu ile encoder okur
- `set_motor_values()` - `"m left right\r"` komutu ile motor kontrolü
- `set_pid_values()` - `"u Kp:Kd:Ki:Ko\r"` komutu ile PID ayarları

### wheel.hpp

**Ne yapar**: Tekerlek verilerini yönetir - encoder ticks'leri radyan'a çevirir.

**Özellikler**:
- `calc_enc_angle()` - Encoder ticks → radyan
- Position ve velocity hesaplama

## 📦 Bağımlılıklar

- `hardware_interface` - ROS2 Control interface
- `libserial` - Serial port kütüphanesi
- `pluginlib` - Plugin sistemi
- `rclcpp` / `rclcpp_lifecycle` - ROS2 C++ API

## 🏗️ Build

```bash
cd ~/robot_ws
colcon build --packages-select my_robot_hardware
source install/setup.bash
```

## ⚙️ Konfigürasyon

ROS2 Control config: `../my_robot_description/ros2_control/my_robot.ros2_control.xacro`

**Parametreler**:
- `device`: Serial port (`/dev/ttyUSB0`)
- `baud_rate`: 57600
- `enc_counts_per_rev`: 3600
- `left_wheel_name` / `right_wheel_name`: Joint isimleri
- `loop_rate`: 30 Hz

## 🔌 Protokol

ROSArduinoBridge protokolü kullanılır. Detaylar: `../my_robot_bringup/arduino/ARDUINO_SERIAL_PROTOCOL.md`

## 🆚 Python Bridge ile Fark

| Özellik | ROS2 Control (Bu paket) | Python Bridge |
|---------|------------------------|---------------|
| Standart ROS2 | ✅ Evet | ❌ Hayır |
| Lifecycle | ✅ Evet | ❌ Hayır |
| Controller entegrasyonu | ✅ Evet | ❌ Hayır |
| Basitlik | ❌ Karmaşık (C++) | ✅ Basit (Python) |

**Not**: İki seçenek de mevcut. İhtiyaca göre kullanın.
