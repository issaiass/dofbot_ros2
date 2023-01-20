import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # CONSTANTS
    package    = 'yahboomcar_gazebo' # the package of the launch file
    world_name = 'empty.world'

    # ENV VARIABLES

    # PATHS
    world_name_path = PathJoinSubstitution([FindPackageShare(package), "worlds", world_name])
    gazebo_client_path = PathJoinSubstitution([FindPackageShare('gazebo_ros'), "launch", 'gazebo.launch.py'])    
    
    # ARGUMENT DECLARATIONS
    arg_world = DeclareLaunchArgument(name='world', default_value=world_name_path, description='path to the world file')

    # LAUNCH FILES TO LAUNCH!

    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_client_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )     

    # NODES TO LAUNCH!

    # Prepare arguments and node as objects
    arguments = [arg_world]
    nodes = []
    launch = [gazebo]
    objects = arguments + nodes + launch
    
    # do launch
    return LaunchDescription(objects)