import launch
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    arrival_listener_node = Node(
        package='circle',
        executable='arrival_listener',
        name='arrival_listener',
        output='screen'
    )

    mission_handler_node = Node(
        package='circle',
        executable='mission_handler',
        name='mission_handler',
        output='screen'
    )

    send_goal_node = Node(
        package='circle',
        executable='send_goal',
        name='send_goal',
        output='screen'
    )

    return launch.LaunchDescription(
        [
            arrival_listener_node,
            mission_handler_node,
            send_goal_node
        ]
    )
 