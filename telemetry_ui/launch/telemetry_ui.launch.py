from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    telemetry_ui = Node(
        package='telemetry_ui',
        executable='telemetry_ui',
        name='telemetry_ui',
        output='screen',
    )

    return LaunchDescription([
        telemetry_ui,
    ])
