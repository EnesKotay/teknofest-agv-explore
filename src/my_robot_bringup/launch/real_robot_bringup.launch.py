"""
Gerçek Robot Bringup Launch Dosyası
Micro-ROS kullanarak STM32 Nucleo ile haberleşme

Bu launch dosyası:
1. micro_ros_agent başlatır (STM32 ile haberleşme için)
2. robot_state_publisher başlatır (URDF -> TF)
3. Odometry TF transform ekler (eğer STM32'de TF yoksa)

Kullanım:
ros2 launch my_robot_bringup real_robot_bringup.launch.py serial_port:=/dev/ttyACM0
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
    
    # Launch Arguments
    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value=TextSubstitution(text="/dev/ttyACM0"),
        description="STM32 Serial port (ST-Link Virtual COM Port)"
    )
    
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate",
        default_value=TextSubstitution(text="460800"),
        description="Serial baud rate for Micro-ROS (recommended: 460800 for STM32)"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time"
    )
    
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # ---------- Micro-ROS Agent ----------
    # STM32'daki Micro-ROS node'larını ROS2 network'e bağlar
    # QoS: Micro-ROS varsayılan QoS kullanır (RELIABLE için topic'ler, BEST_EFFORT için sensor data)
    # Baudrate: 460800 önerilir (STM32 için), 115200 minimum
    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        arguments=[
            "serial",
            "--dev", serial_port,
            "-b", baud_rate,
            "-v6"  # Verbose level 6 (debug)
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
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
    # STM32'dan gelen /odom topic'inden TF yayınlar (DYNAMIC, static değil!)
    # odom_tf_broadcaster, /odom mesajlarını dinleyip TF tree'ye odom -> base_footprint transform'u ekler
    pkg_explore = FindPackageShare("my_robot_explore")
    
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
        declare_use_sim_time,
        micro_ros_agent,
        robot_state_publisher,
        odom_tf_broadcaster,  # Dynamic TF - /odom topic'inden TF yayınlar
    ])
