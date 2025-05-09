from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mars_rover_control',
            executable='motion_command',
            name='motion_command'
        ),
        Node(
            package='mars_rover_control',
            executable='obstacle_detector',
            name='obstacle_detector'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_all',
            output='screen',
            arguments=[
                '/rover_blue_cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/model/rover_blue/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose'
            ]
        ),
    ])
