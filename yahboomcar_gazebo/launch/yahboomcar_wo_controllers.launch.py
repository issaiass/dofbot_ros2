from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # CONSTANTS
    package = 'yahboomcar_gazebo' # the package of the launch file
    world   = 'empty.world'

    # PATHS
    spawn_launch_path = PathJoinSubstitution([FindPackageShare(package), 'launch', 'spawn.launch.py'])
    world_path = PathJoinSubstitution([FindPackageShare(package), 'worlds', world])
    gazebo_path = PathJoinSubstitution([FindPackageShare(package), "launch", 'gazebo.launch.py']) 
    robot_state_publisher_pkg_path = PathJoinSubstitution([FindPackageShare('yahboomcar_description'), 'launch', 'robot_state_publisher.launch.py'])

    # ARGUMENT DECLARATIONS
    arg_world = DeclareLaunchArgument(name='world', default_value=world_path, description='path to the world file')

    # LAUNCH FILES TO LAUNCH!

    # launch robot_state_publisher
    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_publisher_pkg_path))

    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )     
    
    spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_path), 
            launch_arguments={'publish_joints': 'true', 'use_sim_time': 'true'}.items())

    
    # NODES TO LAUNCH!

    # Prepare arguments and node as objects
    arguments = [arg_world]
    nodes = []
    launch = [robot_state_publisher, gazebo, spawn]
    objects = arguments + nodes + launch
    
    # do launch
    return LaunchDescription(objects)