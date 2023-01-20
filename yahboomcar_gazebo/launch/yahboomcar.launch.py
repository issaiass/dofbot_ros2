from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # CONSTANTS
    package = 'yahboomcar_gazebo' # the package of the launch file
    world   = 'empty.world'

    # PATHS
    world_path = PathJoinSubstitution([FindPackageShare(package), 'worlds', world])
    gazebo_path = PathJoinSubstitution([FindPackageShare(package), "launch", 'gazebo.launch.py']) 
    robot_state_publisher_pkg_path = PathJoinSubstitution([FindPackageShare('yahboomcar_description'), 'launch', 'robot_state_publisher.launch.py'])

    # ARGUMENT DECLARATIONS
    arg_world = DeclareLaunchArgument(name='world', default_value=world_path, description='path to the world file')

    # NODES
    spawn = Node(package='gazebo_ros',
                       executable='spawn_entity.py',
                       name='urdf_spawner',
                       arguments=['-topic', '/robot_description',
                                  '-entity', 'robot', '-z', '0.5'],
                       output='screen')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    # LAUNCH FILES
    # launch robot_state_publisher
    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_publisher_pkg_path))

    # launch gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )


    # EVENTS
    # joint state broadcaster event waits to spawn the entity first
    joint_state_broadcaster_spawner_evt = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )

    # robot controller spawner gets up after joint state broadcaster
    robot_controller_spawner_evt = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )


    # Prepare arguments and node as objects
    arguments = [arg_world]
    nodes = [spawn]
    launch = [robot_state_publisher, gazebo]
    evt = [joint_state_broadcaster_spawner_evt, robot_controller_spawner_evt]
    objects = arguments + nodes + launch + evt
    
    # do launch
    return LaunchDescription(objects)