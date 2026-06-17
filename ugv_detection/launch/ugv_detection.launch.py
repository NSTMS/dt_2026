from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('target_fps', default_value='12'),
            DeclareLaunchArgument('camera_width', default_value='640'),
            DeclareLaunchArgument('camera_height', default_value='480'),
            DeclareLaunchArgument('camera_source', default_value='csi'),
            DeclareLaunchArgument('udp_port', default_value='5600'),
            DeclareLaunchArgument('yolo_confidence', default_value='0.65'),
            DeclareLaunchArgument('voting_window_size', default_value='10'),
            DeclareLaunchArgument('voting_min_hits', default_value='6'),
            DeclareLaunchArgument('class_whitelist', default_value=''),
            DeclareLaunchArgument('class_blacklist', default_value=''),
            DeclareLaunchArgument(
                'yolo_model',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('ugv_detection'), 'yolov8n_ground.pt']
                ),
            ),
            DeclareLaunchArgument(
                'tank_params_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('tank_motor'), 'config', 'tank_params.yaml']
                ),
            ),
            SetEnvironmentVariable('TARGET_FPS', LaunchConfiguration('target_fps')),
            SetEnvironmentVariable('CAMERA_WIDTH', LaunchConfiguration('camera_width')),
            SetEnvironmentVariable('CAMERA_HEIGHT', LaunchConfiguration('camera_height')),
            SetEnvironmentVariable('CAMERA_SOURCE', LaunchConfiguration('camera_source')),
            SetEnvironmentVariable('UDP_PORT', LaunchConfiguration('udp_port')),
            SetEnvironmentVariable('YOLO_CONFIDENCE', LaunchConfiguration('yolo_confidence')),
            SetEnvironmentVariable(
                'VOTING_WINDOW_SIZE',
                LaunchConfiguration('voting_window_size'),
            ),
            SetEnvironmentVariable(
                'VOTING_MIN_HITS',
                LaunchConfiguration('voting_min_hits'),
            ),
            SetEnvironmentVariable('CLASS_WHITELIST', LaunchConfiguration('class_whitelist')),
            SetEnvironmentVariable('CLASS_BLACKLIST', LaunchConfiguration('class_blacklist')),
            SetEnvironmentVariable('YOLO_MODEL', LaunchConfiguration('yolo_model')),
            Node(
                package='ugv_detection',
                executable='ugv_detection',
                name='ugv_detection_node',
                output='screen',
            ),
            Node(
                package='tank_motor',
                executable='motor_node',
                name='motor',
                output='screen',
                parameters=[LaunchConfiguration('tank_params_file')],
            ),
            Node(
                package='tank_motor',
                executable='actuator_node',
                name='actuator_node',
                output='screen',
                parameters=[LaunchConfiguration('tank_params_file')],
            ),
        ]
    )
