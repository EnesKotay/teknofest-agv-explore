#!/usr/bin/env python3
"""
Arduino Serial Bridge Node
ROSArduinoBridge protokolü ile uyumlu - geçen senenin kodlarına göre optimize edilmiş

Bu node:
1. /cmd_vel'i dinler ve Arduino'ya motor komutları gönderir
2. Arduino'dan encoder verilerini okur ve /odom yayınlar
3. Emergency button durumunu takip eder (opsiyonel)

Protokol: ROSArduinoBridge formatı (tek karakter komutlar)
- Motor: "m left_ticks right_ticks\r"
- Encoder: "e\r" -> "left right\n"
- Buzzer: "b 1\r" / "b 0\r"
- Battery: "f\r" -> "voltage:percent\n"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import serial
import threading
import time
import math

# ROSArduinoBridge komutları
READ_ENCODERS = b'e\r'
RESET_ENCODERS = b'r\r'
BATTERY_READ = b'f\r'
BUZZER_CONTROL = b'b %d\r'


class ArduinoBridge(Node):
    """
    Arduino ile seri port üzerinden haberleşme node'u
    ROSArduinoBridge protokolü kullanır
    """

    def __init__(self):
        super().__init__("arduino_bridge")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("wheel_separation", 0.30)  # m (robot genişliği)
        self.declare_parameter("wheel_radius", 0.05)  # m
        self.declare_parameter("encoder_counts_per_rev", 3600)  # Encoder çözünürlüğü
        self.declare_parameter("loop_rate", 30.0)  # Hz
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_odom_tf", False)  # TF odom_tf_broadcaster tarafından yayınlanıyor

        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        timeout = self.get_parameter("timeout").value
        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.encoder_counts_per_rev = self.get_parameter("encoder_counts_per_rev").value
        self.loop_rate = self.get_parameter("loop_rate").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").value

        # Odometry hesaplama için
        self.encoder_left_prev = 0
        self.encoder_right_prev = 0
        self.last_encoder_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Thread safety için lock
        self.serial_lock = threading.Lock()

        # Serial port bağlantısı
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Başlangıçta buffer'ı temizle
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            time.sleep(2)  # Arduino'nun başlaması için bekle
            self.get_logger().info(
                f"Arduino'ya bağlandı: {serial_port} @ {baud_rate} baud"
            )
        except Exception as e:
            self.get_logger().fatal(f"Serial port bağlantısı başarısız: {e}")
            raise

        # Encoder'ları reset et (başlangıçta)
        self.reset_encoders()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # TF broadcaster (opsiyonel)
        if self.publish_odom_tf:
            from tf2_ros import TransformBroadcaster
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # Timer: Odometry okuma ve yayınlama
        timer_period = 1.0 / self.loop_rate
        self.odom_timer = self.create_timer(timer_period, self.update_odometry)

        # Son komut zamanı (auto-stop için)
        self.last_cmd_time = None
        self.auto_stop_interval = 2.0  # saniye
        
        # ROS2 clock referansı
        self.clock = self.get_clock()

        self.get_logger().info("Arduino Bridge başlatıldı")

    def reset_encoders(self):
        """Encoder'ları sıfırla"""
        try:
            with self.serial_lock:
                self.serial_conn.write(RESET_ENCODERS)
                time.sleep(0.1)
                response = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if response == "OK":
                    self.get_logger().info("Encoder'lar resetlendi")
                else:
                    self.get_logger().warn(f"Encoder reset yanıtı: {response}")
        except Exception as e:
            self.get_logger().error(f"Encoder reset hatası: {e}")

    def read_encoders(self):
        """
        Arduino'dan encoder değerlerini oku
        Returns: (left_ticks, right_ticks) veya (None, None) hata durumunda
        """
        try:
            with self.serial_lock:
                self.serial_conn.flushInput()
                self.serial_conn.write(READ_ENCODERS)
                time.sleep(0.01)  # Arduino'nun yanıt vermesi için kısa bekleme

                # Yanıt formatı: "left right\n"
                response = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()

                if not response:
                    return None, None

                parts = response.split()
                if len(parts) == 2:
                    left = int(parts[0])
                    right = int(parts[1])
                    return left, right
                else:
                    self.get_logger().warn(f"Geçersiz encoder yanıtı: {response}")
                    return None, None

        except Exception as e:
            self.get_logger().error(f"Encoder okuma hatası: {e}")
            return None, None

    def send_motor_command(self, left_ticks, right_ticks):
        """
        Arduino'ya motor komutu gönder
        Format: "m left_ticks right_ticks\r"
        """
        try:
            cmd = f"m {int(left_ticks)} {int(right_ticks)}\r".encode('ascii')
            with self.serial_lock:
                self.serial_conn.write(cmd)
                # Arduino'dan "OK" yanıtı beklenebilir (opsiyonel)
                # time.sleep(0.001)  # Gerekirse
        except Exception as e:
            self.get_logger().error(f"Motor komutu gönderme hatası: {e}")

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel mesajını al ve motor komutlarına çevir
        Differential drive kinematiği kullanılır
        """
        # Auto-stop timer'ı sıfırla (ROS2 zamanı kullan)
        self.last_cmd_time = self.clock.now()

        # Twist'ten linear ve angular velocity
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Differential drive denklemleri
        # v_left = linear_vel - (angular_vel * wheel_separation / 2)
        # v_right = linear_vel + (angular_vel * wheel_separation / 2)
        v_left = linear_vel - (angular_vel * self.wheel_separation / 2.0)
        v_right = linear_vel + (angular_vel * self.wheel_separation / 2.0)

        # m/s -> rad/s (wheel angular velocity)
        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # rad/s -> ticks/frame (PID controller için)
        # Arduino'da PID_RATE = 30 Hz kullanılıyor (referans kodda)
        pid_rate = 30.0
        ticks_per_frame_left = omega_left * self.encoder_counts_per_rev / (2.0 * math.pi * pid_rate)
        ticks_per_frame_right = omega_right * self.encoder_counts_per_rev / (2.0 * math.pi * pid_rate)

        # Motor komutlarını gönder
        self.send_motor_command(ticks_per_frame_left, ticks_per_frame_right)

    def update_odometry(self):
        """
        Encoder verilerini oku, odometry hesapla ve yayınla
        """
        # Encoder'ları oku
        left_ticks, right_ticks = self.read_encoders()
        if left_ticks is None or right_ticks is None:
            return

        # Zaman hesaplama
        current_time = self.clock.now()
        if self.last_encoder_time is None:
            self.last_encoder_time = current_time
            self.encoder_left_prev = left_ticks
            self.encoder_right_prev = right_ticks
            return

        # Delta time
        dt = (current_time - self.last_encoder_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Encoder farkları
        d_left = left_ticks - self.encoder_left_prev
        d_right = right_ticks - self.encoder_right_prev

        # Ticks -> metrik mesafe
        # Her tick = (2 * pi * wheel_radius) / encoder_counts_per_rev
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.encoder_counts_per_rev
        d_left_m = d_left * meters_per_tick
        d_right_m = d_right * meters_per_tick

        # Ortalama hareket
        d_center = (d_left_m + d_right_m) / 2.0
        d_theta = (d_right_m - d_left_m) / self.wheel_separation

        # Pozisyon güncellemesi
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Theta normalizasyonu [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Hız hesaplama (m/s)
        v_left = d_left_m / dt
        v_right = d_right_m / dt
        linear_vel = (v_left + v_right) / 2.0
        angular_vel = (v_right - v_left) / self.wheel_separation

        # Odometry mesajı oluştur
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Pozisyon
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientasyon (quaternion)
        from geometry_msgs.msg import Quaternion
        odom.pose.pose.orientation = self.quaternion_from_euler(0.0, 0.0, self.theta)

        # Pozisyon kovaryansı (tahmini)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.1  # yaw

        # Hız
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # Hız kovaryansı
        odom.twist.covariance[0] = 0.01  # linear x
        odom.twist.covariance[35] = 0.1  # angular z

        # Yayınla
        self.odom_pub.publish(odom)

        # TF yayınla (opsiyonel)
        if self.publish_odom_tf:
            from geometry_msgs.msg import TransformStamped
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        # Güncellemeleri kaydet
        self.encoder_left_prev = left_ticks
        self.encoder_right_prev = right_ticks
        self.last_encoder_time = current_time

        # Auto-stop kontrolü
        if self.last_cmd_time is not None:
            time_since_last_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
            if time_since_last_cmd > self.auto_stop_interval:
                self.send_motor_command(0, 0)
                self.last_cmd_time = None

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        """Euler açılarından quaternion'a çevir"""
        from geometry_msgs.msg import Quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main():
    rclpy.init()
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node başlatma hatası: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
