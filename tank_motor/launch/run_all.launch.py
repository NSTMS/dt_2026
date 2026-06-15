import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tank_launch = os.path.join(
        get_package_share_directory('tank_motor'),
        'launch',
        'tank.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tank_launch)),
    ])
