import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_name = LaunchConfiguration("world_name")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="True"
    )
    declare_world_name = DeclareLaunchArgument(
        "world_name", default_value="example_walls_world"
    )

    pkg_robot_description = FindPackageShare("my_robot_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    # World
    world_path = PathJoinSubstitution([
        pkg_robot_description, "worlds", "example_walls_world.sdf"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_path],
        }.items(),
    )

    # Robot SDF (spawn)
    model_sdf_path = PathJoinSubstitution([
        pkg_robot_description, "models", "my_robot", "model.sdf"
    ])

    # Robot URDF (RViz/tf için)
    urdf_xacro_path = PathJoinSubstitution([
        pkg_robot_description, "urdf", "my_robot.urdf.xacro"
    ])

    robot_desc = ParameterValue(
        Command(["xacro", " ", urdf_xacro_path]),
        value_type=str
    )

    # Spawn: world adı MUST doğru olmalı (yoksa /world/default/create bekler)
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world", world_name,
            "-name", "my_robot",
            "-file", model_sdf_path,
            "-x", "0", "-y", "0", "-z", "0.0"
        ],
    )

    # Bridge (GZ <-> ROS)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
    )

    # Robot state publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": robot_desc,
            # bazı sürümlerde jsp bunu topic'ten okumayı sever; açık dursun:
            "publish_robot_description": True,
        }],
    )

    # Wheel joint'leri için (RViz RobotModel teker hatalarını keser)
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "rate": 30.0,
            "robot_description": robot_desc
        }],
    )

    # Scan frame: my_robot/laser_link/lidar -> laser_link arasında statik tf (new-style args)
    static_lidar_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "laser_link",
            "--child-frame-id", "my_robot/laser_link/lidar",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world_name,
        gz_sim,
        bridge,
        rsp,
        jsp,
        static_lidar_fix,
        TimerAction(period=4.0, actions=[spawn_entity]),
    ])
