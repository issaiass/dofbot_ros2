from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # CONSTANTS
    package    = 'yahboomcar_description' # the package of the launch file
    viz_base   = 'yahboomcar.rviz' # the visualization loaded

    # PATHS
    rviz_config_path = PathJoinSubstitution([FindPackageShare(package), 'rviz', viz_base])

    # ARGUMENT DECLARATIONS
    arg_rviz = DeclareLaunchArgument(name='rviz', default_value='true', description='Run rviz [true, false]')
    arg_rviz_config_path = DeclareLaunchArgument(name='rviz_config_path', default_value=rviz_config_path, description='Run rviz [true, false]')
    arg_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time [true, false]')
    
    # NODES TO LAUNCH!
    # Launch rviz2
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_path')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )

    # Prepare arguments and node as objects
    arguments = [arg_rviz, arg_rviz_config_path, arg_use_sim_time]
    nodes = [rviz2]
    launch = []
    objects = arguments + nodes + launch
    
    # do launch
    return LaunchDescription(objects)