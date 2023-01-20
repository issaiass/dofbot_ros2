from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # CONSTANTS
    package    = 'yahboomcar_description' # the package of the launch file

    # PATHS
    joint_state_publisher_pkg_path = PathJoinSubstitution([FindPackageShare(package), 'launch', 'joint_state_publisher.launch.py'])
    robot_state_publisher_pkg_path = PathJoinSubstitution([FindPackageShare(package), 'launch', 'robot_state_publisher.launch.py'])    
    rviz2_launch_path = PathJoinSubstitution([FindPackageShare(package), 'launch', 'rviz2.launch.py'])    

    # ARGUMENT DECLARATIONS

    # NODES TO LAUNCH

    # LAUNCH FILES TO LAUNCH
    joint_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(joint_state_publisher_pkg_path))
    robot_state_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_publisher_pkg_path))
    rviz2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz2_launch_path),
                                            launch_arguments={'use_sim_time': 'true', 'rviz': 'true'}.items())

    # Prepare arguments and node as objects
    arguments = []
    nodes = []
    launch = [joint_state_publisher, robot_state_publisher, rviz2_launch]
    objects = arguments + nodes + launch
    
    # do launch
    return LaunchDescription(objects)