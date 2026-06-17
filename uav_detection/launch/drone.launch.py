from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': '/dev/ttyACM0:115200',
            'target_system_id': 1,
            'target_component_id': 1,
            'system_id': 255,
        }],
    )

    detection = Node(
        package='uav_detection',
        executable='uav_detection_pub',
        name='uav_detection',
        output='screen',
    )

    drop_servo = Node(
        package='uav_detection',
        executable='servo_controller',
        name='servo_controller',
        output='screen',
        parameters=[{
            'servo_position': 'closed',   # startup state: 'closed' or 'opened'
        }],
    )

    return LaunchDescription([
        mavros,
        detection,
        drop_servo,
    ])
