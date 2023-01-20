from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # CONSTANTS
    package    = 'yahboomcar_description' # the package of the launch file
    robot_base = 'yahboomcar_X3plus.urdf.xacro' # the xacro or urdf file
    robot_namespace = 'robot' # the robot namespace, you could use /

    # PATHS
    urdf_path = PathJoinSubstitution([FindPackageShare(package), "urdf", robot_base])

    # ARGUMENT DECLARATIONS
    arg_urdf = DeclareLaunchArgument(name='urdf', default_value=urdf_path, description='URDF path')
    arg_ns = DeclareLaunchArgument(name='ns', default_value=robot_namespace, description='robot namespace [/, robot1]')
    arg_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation time [true, false]')

    # NODES TO LAUNCH!
    # launch robot_state_publisher (just launch one or another)
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'robot_description': Command(['xacro', ' ', LaunchConfiguration('urdf'), ' ', 'ns:=', LaunchConfiguration('ns')])
                       }]
    )

    # Prepare arguments and node as objects
    arguments = [arg_urdf, arg_ns, arg_use_sim_time]
    nodes = [robot_state_publisher]
    launch = []
    evt = []
    objects = arguments + nodes + launch + evt
    
    # do launch
    return LaunchDescription(objects)