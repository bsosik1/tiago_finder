from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    voice_control = Node(
        package='tiago_finder',
        executable='voice_control',
        name='voice_control',
        respawn=False,
        respawn_delay=0,
    )
    goal_manager = Node(
        package='tiago_finder',
        executable='goal_manager',
        name='goal_manager'
    )
    ld = LaunchDescription()
    ld.add_action(voice_control)
    ld.add_action(goal_manager)

    return ld
