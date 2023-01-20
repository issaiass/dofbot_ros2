from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    # CONSTANTS

    # PATHS

    # ENV VARIABLES

    # ARGUMENT DECLARATIONS
    arg_publish_joints = DeclareLaunchArgument(name='publish_joints', default_value='true', description='Launch joint_states_publisher [true, false]')

    # NODES TO LAUNCH!
    # launch robot_state_publisher (just launch one or another)
    joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration("publish_joints"))
    )
    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(LaunchConfiguration("publish_joints"))
    )

    # Prepare arguments and node as objects
    arguments = [arg_publish_joints]
    nodes = [joint_state_publisher, joint_state_publisher_gui]
    launch = []
    evt = []
    objects = arguments + nodes + launch + evt
    
    # do launch
    return LaunchDescription(objects)