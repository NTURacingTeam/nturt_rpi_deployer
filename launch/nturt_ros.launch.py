from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare include files
    # node for receiving can signal
    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_receiver.launch.py",
            ]),
        ]),
    )
    # node for sending can signal
    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_sender.launch.py",
            ]),
        ]),
    )
    # node for parsing can signal
    can_parser = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nturt_can_parser"),
                "nturt_can_parser.launch.py",
            ]),
        ]),
    )
    # node for sneding messages to remote server
    push_to_control_tower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nturt_push_to_control_tower"),
                "nturt_push_to_control_tower.launch.py",
            ]),
        ]),
    )

    includes = [
        socket_can_receiver,
        socket_can_sender,
        can_parser,
        push_to_control_tower,
    ]

    # declare nodes
    # node for recording gps signal 
    gps_node = Node(
        package="nturt_gps",
        executable="nturt_gps",
        output="both",
    )
    led_controller_node = Node(
        package="nturt_led_controller",
        executable="nturt_led_controller_node",
        output="both",
    )

    nodes = [
        gps_node,
        led_controller_node,
    ]

    return LaunchDescription(includes + nodes)
