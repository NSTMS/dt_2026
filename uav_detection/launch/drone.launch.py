from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    rc_listener = Node(
        package='uav_detection',
        executable='rc_listener_node',
        name='rc_listener',
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
        DeclareLaunchArgument('target_fps', default_value='15'),
        DeclareLaunchArgument('camera_source', default_value='csi'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        DeclareLaunchArgument('udp_port', default_value='5000'),
        DeclareLaunchArgument('stream_host', default_value='10.199.220.67'),
        DeclareLaunchArgument('stream_port', default_value='5600'),
        DeclareLaunchArgument('yolo_confidence', default_value='0.5'),
        DeclareLaunchArgument('class_whitelist', default_value=''),
        DeclareLaunchArgument('class_blacklist', default_value=''),
        DeclareLaunchArgument(
            'class_name_map',
            default_value=(
                'tico:maulch,polonez:polonez,lambo:ferrari,bus:autobus,tir:tir,'
                'czolg_zielony:T-90,czolg_bialy:T-62,wyrzutnia:pansir,'
                'humvee:humvee,radar:radar'
            ),
        ),
        DeclareLaunchArgument(
            'yolo_model',
            default_value=PathJoinSubstitution(
                [FindPackageShare('uav_detection'), 'yolov8n_aerial.pt']
            ),
        ),
        SetEnvironmentVariable(
            'GST_PLUGIN_PATH',
            '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0',
        ),
        SetEnvironmentVariable('TARGET_FPS', LaunchConfiguration('target_fps')),
        SetEnvironmentVariable('CAMERA_SOURCE', LaunchConfiguration('camera_source')),
        SetEnvironmentVariable('CAMERA_WIDTH', LaunchConfiguration('camera_width')),
        SetEnvironmentVariable('CAMERA_HEIGHT', LaunchConfiguration('camera_height')),
        SetEnvironmentVariable('UDP_PORT', LaunchConfiguration('udp_port')),
        SetEnvironmentVariable('STREAM_HOST', LaunchConfiguration('stream_host')),
        SetEnvironmentVariable('STREAM_PORT', LaunchConfiguration('stream_port')),
        SetEnvironmentVariable('YOLO_CONFIDENCE', LaunchConfiguration('yolo_confidence')),
        SetEnvironmentVariable('CLASS_WHITELIST', LaunchConfiguration('class_whitelist')),
        SetEnvironmentVariable('CLASS_BLACKLIST', LaunchConfiguration('class_blacklist')),
        SetEnvironmentVariable('CLASS_NAME_MAP', LaunchConfiguration('class_name_map')),
        SetEnvironmentVariable('YOLO_MODEL', LaunchConfiguration('yolo_model')),
        mavros,
        detection,
        rc_listener,
        drop_servo,
    ])
