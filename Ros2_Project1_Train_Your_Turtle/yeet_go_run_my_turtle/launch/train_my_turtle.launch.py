

from launch import LaunchDescription


from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
        ),

        Node(
            package='yeet_go_run_my_turtle',
            node_executable='trainer',
            name='trainer',
            output='screen'),
    ])
