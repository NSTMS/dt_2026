from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    # Zabija stare instancje i restartuje lguard przed startem nodów
    cleanup = ExecuteProcess(
        cmd=['bash', '-c', 'kill $(lsof -t /dev/gpiochip4) 2>/dev/null; sudo killall lguard 2>/dev/null; sleep 1'],
        output='screen',
        name='gpio_cleanup',
    )

    motor = Node(
        package='tank_motor',
        executable='motor_node',
        name='motor',
        output='screen',
    )

    actuator_node = Node(
        package='tank_motor',
        executable='actuator_node',
        name='actuator_node',
        output='screen',
    )

    # Uruchom nody dopiero po zakończeniu cleanup
    start_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=cleanup,
            on_exit=[motor, actuator_node],
        )
    )

    return LaunchDescription([
        cleanup,
        start_after_cleanup,
    ])