from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frequency = LaunchConfiguration('frequency')

    return LaunchDescription([

        DeclareLaunchArgument(
            'frequency',
            default_value='5',
            description='Control loop frequency in Hz'
        ),

        Node(
            package='gait_controller',
            executable='gait_main_node',
            name='gait_main_node',
            output='screen',
            parameters=[
                {'frequency': frequency}
            ]
        ),

        Node(
            package='gait_controller',
            executable='gait_sequencer_time_action_node',
            name='gait_sequencer_time_action_node',
            output='screen',
            parameters=[
                {'frequency': frequency}
            ]
        ),

        Node(
            package='gait_controller',
            executable='pump_frequency_publisher_node',
            name='pump_frequency_publisher_node',
            output='screen',
            parameters=[
                {'frequency': frequency}
            ]
        ),

    ])
