from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # CONSTANTS
    x, y, z = 0.0, 0.0, 0.0

    # PATHS

    # ARGUMENT DECLARATIONS
    arg_x = DeclareLaunchArgument(name='x', default_value=x, description='x pos [0.0]')
    arg_y = DeclareLaunchArgument(name='y', default_value=y, description='y pos [0.0]')    
    arg_z = DeclareLaunchArgument(name='z', default_value=z, description='z pos [0.0]')

    # LAUNCH FILES TO LAUNCH!

    # NODES TO LAUNCH!
    # Spawn the robot in Gazebo
    spawn_robot = Node(package='gazebo_ros',
                       executable='spawn_entity.py',
                       name='urdf_spawner',
                       arguments=['-topic', '/robot_description',
                                  '-entity', 'robot', 
                                  '-x', LaunchConfiguration('x'), 
                                  '-y', LaunchConfiguration('y'),
                                  '-z', LaunchConfiguration('z')],
                       output='screen')

    # Prepare arguments and node as objects
    arguments = [arg_x, arg_y, arg_z]
    nodes = [spawn_robot]
    launch = []
    objects = arguments + nodes + launch

    # do launch
    return LaunchDescription(objects)
