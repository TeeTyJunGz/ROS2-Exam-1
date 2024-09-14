from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_description = LaunchDescription()

    turtle_name_controller = LaunchConfiguration('turtle_name_controller')
    controller_launch_arg = DeclareLaunchArgument(
        'turtle_name_controller',
        default_value = 'turtle1'
    )
    
    launch_description.add_action(controller_launch_arg)

    turtlesim_node = Node(
            package='turtlesim_plus',
            namespace='teleop',
            executable='turtlesim_plus_node.py',
            name='turtlesim'

        )
    launch_description.add_action( turtlesim_node )
    
    kill_turtle = ExecuteProcess(
        cmd=["ros2 service call /teleop/remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
             shell =True
        )
    launch_description.add_action( kill_turtle )


    turtlesim_copy_node = Node(
            package='turtlesim_plus',
            namespace='copy',
            executable='turtlesim_plus_node.py',
            name='turtlesim'

        )
    launch_description.add_action( turtlesim_copy_node )
    
    kill_turtle = ExecuteProcess(
        cmd=["ros2 service call /copy/remove_turtle turtlesim/srv/Kill \"name: 'turtle1'\""],
             shell =True
        )
    launch_description.add_action( kill_turtle )
    
    package_name = 'turtlesim_promax'
    executable_name = ['teleop_controller', 'teleop_scheduler', 'coppy_controller', 'coppy_scheduler']
    copy_turtle_name = ['Foxy','Noetic','Humble','Iron']

    for i in range(len(executable_name)):
        
        if executable_name[i] == 'teleop_controller':
            
            teleop_controller = Node(
            package = package_name,
            namespace = turtle_name_controller,
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            parameters=[
                {'pizza_max': 20},
                {'Kp': 1.5},
                {'copy_name': copy_turtle_name},
                {'mon_name': 'monitor1'}
            ]
            )
            launch_description.add_action(teleop_controller)
                       
        elif executable_name[i] == 'teleop_scheduler':
            
            teleop_sch = Node(
            package = package_name,
            namespace = turtle_name_controller,
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            )
            launch_description.add_action(teleop_sch)

        elif executable_name[i] == 'coppy_scheduler':
            
            copy_sch = Node(
            package = package_name,
            namespace = 'copy',
            executable = executable_name[i] + '.py',
            name = executable_name[i],
            )
            launch_description.add_action(copy_sch)
            
        elif executable_name[i] == 'coppy_controller':
            for j in range(len(copy_turtle_name)):
            
                copy_control = Node(
                package = package_name,
                namespace = copy_turtle_name[j],
                executable = executable_name[i] + '.py',
                name = copy_turtle_name[j] + '_' + executable_name[i],
                )
                launch_description.add_action(copy_control)

            
        # else:
        #     noise_generator = Node(
        #     package = package_name,
        #     namespace = '',
        #     executable = executable_name[i] + '.py',
        #     name = executable_name[i],
        # )
        
    return launch_description