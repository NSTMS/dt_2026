import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tank_share = get_package_share_directory('tank_motor')
    params_file = os.path.join(tank_share, 'config', 'tank_params.yaml')

    teleop = LaunchConfiguration('teleop')
    detection = LaunchConfiguration('detection')

    cleanup = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'kill $(lsof -t /dev/gpiochip4) 2>/dev/null; sudo killall lguard 2>/dev/null; sleep 1',
        ],
        output='screen',
        name='gpio_cleanup',
    )

    motor = Node(
        package='tank_motor',
        executable='motor_node',
        name='motor',
        output='screen',
        parameters=[params_file],
    )

    actuator = Node(
        package='tank_motor',
        executable='actuator_node',
        name='actuator_node',
        output='screen',
        parameters=[params_file],
    )

    # Teleop w osobnym procesie z dziedziczonym TTY — jedyny niezawodny sposób z launch
    keyboard = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep 2 && exec ros2 run tank_motor keyboard_controller --ros-args --params-file {params_file}',
        ],
        output='screen',
        condition=IfCondition(teleop),
    )

    ugv_detection = Node(
        package='ugv_detection',
        executable='ugv_detection_pub',
        name='ugv_detection',
        output='screen',
        condition=IfCondition(detection),
    )

    start_stack = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[motor, actuator, keyboard, ugv_detection],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop',
            default_value='true',
            description='Uruchom keyboard_controller (wymaga interaktywnego terminala SSH)',
        ),
        DeclareLaunchArgument(
            'detection',
            default_value='true',
            description='Uruchom ugv_detection_pub (YOLO obciąża CPU)',
        ),
        cleanup,
        start_stack,
    ])
