from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('target_fps', default_value='12'),
            DeclareLaunchArgument('camera_width', default_value='1280'),
            DeclareLaunchArgument('camera_height', default_value='720'),
            DeclareLaunchArgument('camera_source', default_value='csi'),
            DeclareLaunchArgument('udp_port', default_value='5600'),
            DeclareLaunchArgument('yolo_confidence', default_value='0.65'),
            DeclareLaunchArgument('qr_confirm_count', default_value='3'),
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
            DeclareLaunchArgument('enable_qr_reader', default_value='true'),
            SetEnvironmentVariable(
                'GST_PLUGIN_PATH',
                '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0',
            ),
            SetEnvironmentVariable('TARGET_FPS', LaunchConfiguration('target_fps')),
            SetEnvironmentVariable('CAMERA_WIDTH', LaunchConfiguration('camera_width')),
            SetEnvironmentVariable('CAMERA_HEIGHT', LaunchConfiguration('camera_height')),
            SetEnvironmentVariable('CAMERA_SOURCE', LaunchConfiguration('camera_source')),
            SetEnvironmentVariable('UDP_PORT', LaunchConfiguration('udp_port')),
            SetEnvironmentVariable('YOLO_CONFIDENCE', LaunchConfiguration('yolo_confidence')),
            SetEnvironmentVariable('QR_CONFIRM_COUNT', LaunchConfiguration('qr_confirm_count')),
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
                package='qr_reader',
                executable='qr_node',
                name='qr_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_qr_reader')),
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
