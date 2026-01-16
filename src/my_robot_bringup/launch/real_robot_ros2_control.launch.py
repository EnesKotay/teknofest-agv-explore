"""
Gerçek Robot Bringup Launch Dosyası - ROS2 Control ile
Teknofest-AGV değerleriyle güncellendi (Raspberry Pi 5 + ROS2 Jazzy)

Bu launch dosyası:
1. ROS2 Control hardware interface'i başlatır (Arduino ile haberleşme)
2. Controller manager'ı başlatır
3. Diff drive controller'ı spawn eder
4. Robot state publisher başlatır (URDF -> TF)
5. Joint state broadcaster spawn eder

Kullanım:
ros2 launch my_robot_bringup real_robot_ros2_control.launch.py device:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_desc = FindPackageShare("my_robot_description")
    
    # Launch Arguments
    declare_device = DeclareLaunchArgument(
        "device",
        default_value=TextSubstitution(text="/dev/ttyUSB0"),
        description="Arduino Serial port (/dev/ttyUSB0, /dev/ttyACM0, vb.)"
    )
    
    device = LaunchConfiguration("device")
    
    # URDF dosyasını xacro ile oluştur
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_desc, "urdf", "my_robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # Controller config dosyası
    robot_controllers = PathJoinSubstitution(
        [pkg_bringup, "config", "my_robot_controllers.yaml"]
    )
    
    # ---------- ROS2 Control Node ----------
    # Hardware interface'i başlatır, controller manager'ı çalıştırır
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    
    # ---------- Robot State Publisher (URDF -> TF) ----------
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # ---------- Joint State Broadcaster Spawner ----------
    # Joint state'leri yayınlar (/joint_states topic)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # ---------- Diff Drive Controller Spawner ----------
    # Motor kontrolü yapar, odometry yayınlar
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )
    
    # ---------- Event Handlers ----------
    # Joint state broadcaster başladıktan sonra diff drive controller'ı başlat
    delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    return LaunchDescription([
        declare_device,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
