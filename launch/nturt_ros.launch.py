from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "is_realtime",
            default_value="false",
            description="Whether to run using real-time scheduler.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "password",
            default_value="raspberry",
            description="The password of the user to get root permission using sudo to setup can bus.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "can_bitrate",
            default_value="500000",
            description="The bitrate of the can.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "gps_port",
            default_value="/dev/ttyUSB0",
            description="The port of the gps receiver.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "gps_baudrate",
            default_value="115200",
            description="The baudrate of the gps receiver.",
        )
    )

    # initialize arguments
    is_realtime = LaunchConfiguration("is_realtime")
    password = LaunchConfiguration("password")
    can_bitrate = LaunchConfiguration("can_bitrate")
    gps_port = LaunchConfiguration("gps_port")
    gps_baudrate = LaunchConfiguration("gps_baudrate")

    # declare include files
    # node for transceiving can signal
    socket_can_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nturt_can_parser"),
                "socket_can_bridge.launch.py",
            ]),
        ]),
        launch_arguments={
            "is_realtime": is_realtime,
            "password": password,
            "bitrate": can_bitrate,
        }.items(),
    )

    includes = [
        socket_can_bridge,
    ]

    # declare nodes
    # node for recording ros bag
    bag_recorder_node = Node(
        package="nturt_bag_recorder",
        executable="nturt_bag_recorder_node.py",
        output="both",
    )
    # node for receiving gps signal 
    gps_node = Node(
        package="nturt_nmea_navsat_driver",
        executable="nmea_ntrip_driver",
        output="both",
        parameters=[{
            "port": gps_port,
            "baud": gps_baudrate,
        }]
    )
    # node for monitoring system stats
    system_stats_monitor_node = Node(
        package="nturt_rpi_deployer",
        executable="system_stats_monitor_node",
        output="both",
    )

    nodes = [
        bag_recorder_node,
        gps_node,
        system_stats_monitor_node,
    ]

    return LaunchDescription(arguments + includes + nodes)
