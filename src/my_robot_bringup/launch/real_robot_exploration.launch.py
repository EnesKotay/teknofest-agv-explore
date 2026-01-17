"""
Gerçek Robot Otonom Keşif Launch Dosyası
Raspberry Pi için optimize edilmiş (Headless - GUI yok)

Bu launch dosyası:
1. real_robot_ros2_control.launch.py'yi include eder (ROS2 Control hardware interface, TF, vb.)
2. SLAM Toolbox ile harita oluşturur
3. Nav2 ile navigasyon sağlar
4. frontier_explorer ile otonom keşif yapar
5. RViz2 opsiyonel (varsayılan: kapalı - headless için)

Kullanım:
ros2 launch my_robot_bringup real_robot_exploration.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
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
    
    declare_laser_filter_config = DeclareLaunchArgument(
        "laser_filter_config",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "laser_filter.yaml"]),
        description="Laser filter config file"
    )
    
    declare_ekf_config = DeclareLaunchArgument(
        "ekf_config_file",
        default_value=PathJoinSubstitution([pkg_bringup, "config", "ekf.yaml"]),
        description="EKF config file"
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value=TextSubstitution(text="false"),  # Varsayılan: kapalı (Raspberry Pi headless için)
        description="Launch RViz2 for visualization (requires GUI/X11 forwarding)"
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([pkg_bringup, "rviz", "exploration.rviz"]),
        description="RViz config file (only used if use_rviz:=true)"
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
    laser_filter_config = LaunchConfiguration("laser_filter_config")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    
    # ---------- Include: real_robot_ros2_control.launch.py ----------
    # ROS2 Control hardware interface, robot_state_publisher, diff_drive_controller
    # Teknofest-AGV ile uyumlu (Raspberry Pi 5 + ROS2 Jazzy)
    real_robot_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, "launch", "real_robot_ros2_control.launch.py"])
        ),
        launch_arguments={
            "device": serial_port,  # ROS2 Control'de "device" parametresi kullanılıyor
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
    
    # ---------- Laser Filter (Box Filter) ----------
    # Robot gövdesini ve Karakuri'yi filtreler, sadece gerçek engelleri algılar
    laser_filter_node = TimerAction(
        period=1.5,  # 1.5 saniye bekle (LIDAR çalışmaya başlasın)
        actions=[
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="laser_filter_node",
                parameters=[laser_filter_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("scan", "/scan"),  # Orijinal scan topic
                    ("scan_filtered", "/scan_filtered"),  # Filtrelenmiş scan topic
                ],
                output="screen",
            )
        ]
    )
    
    # ---------- EKF (Extended Kalman Filter) ----------
    # Odometry ve IMU verilerini birleştirir, daha doğru pozisyon tahmini sağlar
    # Teknofest-AGV'de kullanılıyor
    ekf_node = TimerAction(
        period=3.0,  # 3 saniye bekle (ROS2 Control ve odometry hazır olsun)
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config_file, {"use_sim_time": use_sim_time}],
            )
        ]
    )
    
    # ---------- SLAM Toolbox ----------
    slam_toolbox_launch = TimerAction(
        period=4.0,  # 4 saniye bekle (TF tree ve EKF hazır olsun)
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
        period=10.0,  # 10 saniye bekle (SLAM map frame oluştursun, EKF hazır olsun)
        actions=navigation
    )
    
    # ---------- Frontier Explorer ----------
    explore_node = TimerAction(
        period=14.0,  # 14 saniye bekle (Nav2, SLAM ve EKF hazır olsun)
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
    
    # ---------- RViz2 (Opsiyonel - sadece GUI/X11 forwarding varsa) ----------
    # Raspberry Pi headless için varsayılan: kapalı
    # GUI erişimi varsa: use_rviz:=true ile başlatılabilir
    rviz_node = TimerAction(
        period=16.0,  # 16 saniye bekle (tüm sistem hazır olsun)
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
                condition=IfCondition(use_rviz)  # Sadece use_rviz:=true ise başlat
            )
        ]
    )
    
    return LaunchDescription([
        # Launch Arguments
        declare_serial_port,
        declare_baud_rate,
        declare_wheel_separation,
        declare_wheel_radius,
        declare_encoder_counts_per_rev,
        declare_timeout,
        declare_use_sim_time,
        declare_autostart,
        declare_nav2_params,
        declare_slam_params,
        declare_explore_params,
        declare_laser_filter_config,
        declare_ekf_config,
        declare_use_rviz,
        declare_rviz_config,
        declare_lidar_port,
        
        # Robot Control (ROS2 Control)
        real_robot_ros2_control,      # 0 saniye - anında başlar
        
        # Sensors
        lidar_node,                    # 1 saniye sonra - RPLIDAR A2
        laser_filter_node,             # 1.5 saniye sonra - Box filter (robot gövdesini filtreler)
        
        # Localization
        ekf_node,                      # 3 saniye sonra - EKF (odometry + IMU fusion)
        
        # SLAM & Navigation
        slam_toolbox_launch,           # 4 saniye sonra - SLAM Toolbox
        nav2_navigation,               # 10 saniye sonra - Nav2 Navigation Stack
        
        # Exploration
        explore_node,                  # 14 saniye sonra - Frontier Explorer
        
        # Visualization (Opsiyonel - varsayılan: kapalı)
        rviz_node,                     # 16 saniye sonra - Sadece use_rviz:=true ise
    ])
