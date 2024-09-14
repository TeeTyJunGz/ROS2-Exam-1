from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    launch_description = LaunchDescription()

    rate_mux = LaunchConfiguration('rate')
    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value = '5.0'
        )
    launch_description.add_action( rate_launch_arg )

    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace='',
            executable='turtlesim_plus_node.py',
            name='turtlesim'

        )
    launch_description.add_action( turtlesim_node )

    kill_turtle = ExecuteProcess(
        cmd=["ros2 service call /remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
             shell =True
        )
    launch_description.add_action( kill_turtle )


    spawn_turtle = ExecuteProcess(
        cmd=["ros2 service call /spawn_turtle turtlesim/srv/Spawn \"{x: 0.0, y: 0.0, theta: 0.0, name: 'foxy_65'}\""],
        shell = True
    )
    launch_description.add_action( spawn_turtle )

    # spawn_turtle_2 = ExecuteProcess(
    #     cmd=["ros2 service call /spawn_turtle turtlesim/srv/Spawn \"{x: 2.0, y: 2.0, theta: 0.0, name: 'foxy_41'}\""],
    #     shell = True
    # )
    # launch_description.add_action( spawn_turtle_2 )

    return launch_description
