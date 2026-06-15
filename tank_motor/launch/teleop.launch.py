import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    tank_share = get_package_share_directory('tank_motor')
    params_file = os.path.join(tank_share, 'config', 'tank_params.yaml')

    keyboard = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tank_motor', 'keyboard_controller',
            '--ros-args', '--params-file', params_file,
        ],
        output='screen',
    )

    return LaunchDescription([keyboard])
