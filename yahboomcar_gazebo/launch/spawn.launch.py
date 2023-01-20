from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # CONSTANTS

    # PATHS

    # ARGUMENT DECLARATIONS

    # LAUNCH FILES TO LAUNCH!

    # NODES TO LAUNCH!
    # Spawn the robot in Gazebo
    spawn_robot = Node(package='gazebo_ros',
                       executable='spawn_entity.py',
                       name='urdf_spawner',
                       arguments=['-topic', '/robot_description',
                                  '-entity', 'robot', '-z', '0.5'],
                       output='screen')

    # Prepare arguments and node as objects
    arguments = []
    nodes = [spawn_robot]
    launch = []
    objects = arguments + nodes + launch

    # do launch
    return LaunchDescription(objects)
