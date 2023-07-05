from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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
            default_value="1000000",
            description="The bitrate of the can.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "gps_port",
            default_value="/dev/ttyACM0",
            description="The port of the gps receiver.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "gps_baudrate",
            default_value="9600",
            description="The baudrate of the gps receiver.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "control_tower_ip",
            default_value="140.112.14.14",
            description="The ip of the control tower.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "control_tower_port",
            default_value="'21543'",
            description="The port of the control tower.",
        )
    )

    # initialize arguments
    is_realtime = LaunchConfiguration("is_realtime")
    password = LaunchConfiguration("password")
    can_bitrate = LaunchConfiguration("can_bitrate")
    gps_port = LaunchConfiguration("gps_port")
    gps_baudrate = LaunchConfiguration("gps_baudrate")
    control_tower_ip = LaunchConfiguration("control_tower_ip")
    control_tower_port = LaunchConfiguration("control_tower_port")

    #envirnoment variable
    environment_variables = []
    environment_variables.append(
        SetEnvironmentVariable(name="DISPLAY", value=":0")
    )

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
    # node for sneding messages to remote server
    push_to_control_tower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nturt_push_to_control_tower"),
                "nturt_push_to_control_tower.launch.py",
            ]),
        ]),
        launch_arguments={
            "ip": control_tower_ip,
            "port": control_tower_port,
        }.items(),
    )

    includes = [
        socket_can_bridge,
        push_to_control_tower,
    ]

    # declare nodes
    # node for recording ros bag
    bag_recorder_node = Node(
        package="nturt_bag_recorder",
        executable="nturt_bag_recorder_node",
        output="both",
    )
    # node for receiving gps signal 
    gps_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        output="both",
        parameters=[{
            "port": gps_port,
            "baud": gps_baudrate,
        }]
    )
    # node for controlling led
    led_controller_node = Node(
        package="nturt_led_controller",
        executable="nturt_led_controller_node",
        output="both",
    )
    # node for displaying screen
    screen_controller_node = Node(
        package="nturt_screen_controller",
        executable="nturt_screen_controller_node",
        output="both",
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
        led_controller_node,
        screen_controller_node,
        system_stats_monitor_node,
    ]

    return LaunchDescription(arguments + environment_variables + includes + nodes)
