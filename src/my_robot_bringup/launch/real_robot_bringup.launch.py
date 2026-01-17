"""
Gerçek Robot Bringup Launch Dosyası
Arduino ile seri port haberleşme (ROSArduinoBridge protokolü)

Bu launch dosyası:
1. arduino_bridge node'unu başlatır (cmd_vel -> motor komutları, encoder -> odometry)
2. robot_state_publisher başlatır (URDF -> TF)
3. Odometry TF transform ekler

⚠️ ÖNEMLİ: Bu launch arduino_bridge.py kullanır, ROS2 Control KULLANMAZ!
           ASLA ROS2 Control ile birlikte çalıştırmayın (serial port çakışması)!
           Bu launch sadece test amaçlıdır, production için ROS2 Control kullanın!

Kullanım:
ros2 launch my_robot_bringup real_robot_bringup.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    pkg_explore = FindPackageShare("my_robot_explore")
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="Arduino Serial port (/dev/ttyUSB0, /dev/ttyACM0, vb.)"
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value=TextSubstitution(text="57600"),
        description="Serial baud rate (57600 önerilir - ROSArduinoBridge varsayılanı)"
    )
    
    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value=TextSubstitution(text="0.43"),  # Teknofest-AGV değeri (gerçek robot ölçüsü)
        description="Tekerlekler arası mesafe (metre)"
    )
    
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value=TextSubstitution(text="0.050"),  # Teknofest-AGV değeri (gerçek robot ölçüsü)
        description="Tekerlek yarıçapı (metre)"
    )
    
    declare_encoder_counts_per_rev = DeclareLaunchArgument(
        "encoder_counts_per_rev",
        default_value=TextSubstitution(text="4000"),  # Teknofest-AGV değeri (gerçek encoder çözünürlüğü)
        description="Encoder çözünürlüğü (bir turdaki tick sayısı)"
    )
    
    declare_timeout = DeclareLaunchArgument(
        "timeout",
        default_value=TextSubstitution(text="1.0"),
        description="Serial port timeout (saniye)"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time"
    )
    
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius = LaunchConfiguration("wheel_radius")
    encoder_counts_per_rev = LaunchConfiguration("encoder_counts_per_rev")
    timeout = LaunchConfiguration("timeout")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # ---------- Arduino Bridge Node ----------
    # /cmd_vel'i dinler, motor komutları gönderir
    # Encoder verilerini okur, /odom yayınlar
    arduino_bridge = Node(
        package="my_robot_explore",
        executable="arduino_bridge",
        name="arduino_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"serial_port": serial_port},
            {"baud_rate": baud_rate},
            {"timeout": timeout},
            {"wheel_separation": wheel_separation},
            {"wheel_radius": wheel_radius},
            {"encoder_counts_per_rev": encoder_counts_per_rev},
            {"loop_rate": 30.0},  # Hz
            {"odom_frame": "odom"},
            {"base_frame": "base_footprint"},
            {"publish_odom_tf": False},  # odom_tf_broadcaster kullanılacak
        ],
    )
    
    # ---------- Robot State Publisher (URDF -> TF) ----------
    urdf_xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"])
    robot_desc = ParameterValue(
        Command(["xacro", " ", urdf_xacro_path]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_desc}
        ],
    )
    
    # ---------- Dynamic TF: odom -> base_footprint ----------
    # arduino_bridge'den gelen /odom topic'inden TF yayınlar
    odom_tf_broadcaster = Node(
        package="my_robot_explore",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odom_topic": "/odom"},
            {"odom_frame": "odom"},
            {"base_frame": "base_footprint"},
        ],
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_wheel_separation,
        declare_wheel_radius,
        declare_encoder_counts_per_rev,
        declare_timeout,
        declare_use_sim_time,
        arduino_bridge,
        robot_state_publisher,
        odom_tf_broadcaster,
    ])
