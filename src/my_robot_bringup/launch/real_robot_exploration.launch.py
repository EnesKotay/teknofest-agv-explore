"""
Gerçek Robot Otonom Keşif Launch Dosyası
Arduino + Serial Port + LIDAR

Bu launch dosyası:
1. real_robot_bringup.launch.py'yi include eder (Arduino bridge, TF, vb.)
2. SLAM Toolbox ile harita oluşturur
3. Nav2 ile navigasyon sağlar
4. frontier_explorer ile otonom keşif yapar
5. RViz2 ile görselleştirme yapar

Kullanım:
ros2 launch my_robot_bringup real_robot_exploration.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Paket dizinleri
    pkg_bringup = FindPackageShare("my_robot_bringup")
    pkg_slam = FindPackageShare("slam_toolbox")
    
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
        default_value=TextSubstitution(text="0.30"),
        description="Tekerlekler arası mesafe (metre)"
    )
    
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value=TextSubstitution(text="0.05"),
        description="Tekerlek yarıçapı (metre)"
    )
    
    declare_encoder_counts_per_rev = DeclareLaunchArgument(
        "encoder_counts_per_rev",
        default_value=TextSubstitution(text="3600"),
        description="Encoder çözünürlüğü (bir turdaki tick sayısı)"
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value=TextSubstitution(text="false"),
        description="Use simulation time"
    )
    
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value=TextSubstitution(text="true"),
        description="Automatically startup the nav2 stack"
    )
    
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "nav2_params.yaml"]),
        description="Nav2 params yaml"
    )
    
    declare_slam_params = DeclareLaunchArgument(
        "slam_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "mapper_params_online_async.yaml"]),
        description="SLAM params yaml"
    )
    
    declare_explore_params = DeclareLaunchArgument(
        "explore_params_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "explore_params.yaml"]),
        description="Exploration params yaml"
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_bringup, "rviz", "exploration.rviz"]),
        description="RViz config file"
    )
    
    # Launch Configurations
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius = LaunchConfiguration("wheel_radius")
    encoder_counts_per_rev = LaunchConfiguration("encoder_counts_per_rev")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    explore_params_file = LaunchConfiguration("explore_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    
    # ---------- Include: real_robot_bringup.launch.py ----------
    # Arduino bridge, robot_state_publisher, odom_tf_broadcaster
    real_robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, "launch", "real_robot_bringup.launch.py"])
        ),
        launch_arguments={
            "serial_port": serial_port,
            "baud_rate": baud_rate,
            "wheel_separation": wheel_separation,
            "wheel_radius": wheel_radius,
            "encoder_counts_per_rev": encoder_counts_per_rev,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    
    # ---------- LIDAR Node (RPLIDAR A2) ----------
    # Not: Arduino /dev/ttyUSB0 ise, LIDAR genellikle /dev/ttyUSB1 olur
    declare_lidar_port = DeclareLaunchArgument(
        "lidar_port",
        default_value=TextSubstitution(text="/dev/ttyUSB1"),
        description="RPLIDAR A2 USB port (Arduino farklı port'ta olabilir)"
    )
    
    lidar_port = LaunchConfiguration("lidar_port")
    
    lidar_node = TimerAction(
        period=1.0,  # 1 saniye bekle (TF tree hazır olsun)
        actions=[
            Node(
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                parameters=[
                    {"serial_port": lidar_port},
                    {"serial_baudrate": 256000},  # RPLIDAR A2 için 256000 baudrate
                    {"frame_id": "laser_link"},
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            )
        ]
    )
    
    # ---------- SLAM Toolbox ----------
    slam_toolbox_launch = TimerAction(
        period=2.0,  # 2 saniye bekle (TF tree hazır olsun)
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": use_sim_time,
                }.items()
            )
        ]
    )
    
    # ---------- Nav2 Navigation Stack ----------
    # Manual node declarations (docking_server hariç)
    nav_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor"
    ]
    
    navigation = []
    
    navigation.append(Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[nav2_params_file, {"use_sim_time": use_sim_time}]
    ))
    
    navigation.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": nav_nodes},
        ],
    ))
    
    nav2_navigation = TimerAction(
        period=4.0,  # 4 saniye bekle (SLAM hazır olsun)
        actions=navigation
    )
    
    # ---------- Frontier Explorer ----------
    explore_node = TimerAction(
        period=6.0,  # 6 saniye bekle (Nav2 hazır olsun)
        actions=[
            Node(
                package="my_robot_explore",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[explore_params_file, {"use_sim_time": use_sim_time}],
            )
        ]
    )
    
    # ---------- RViz2 ----------
    rviz_node = TimerAction(
        period=8.0,  # 8 saniye bekle (sistem hazır olsun)
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen"
            )
        ]
    )
    
    return LaunchDescription([
        declare_serial_port,
        declare_baud_rate,
        declare_wheel_separation,
        declare_wheel_radius,
        declare_encoder_counts_per_rev,
        declare_use_sim_time,
        declare_autostart,
        declare_nav2_params,
        declare_slam_params,
        declare_explore_params,
        declare_rviz_config,
        declare_lidar_port,
        
        real_robot_bringup,      # 0 saniye - anında başlar
        lidar_node,              # 1 saniye sonra - RPLIDAR A2
        slam_toolbox_launch,     # 2 saniye sonra
        nav2_navigation,         # 4 saniye sonra
        explore_node,            # 6 saniye sonra
        rviz_node,               # 8 saniye sonra
    ])
