from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions

def generate_launch_description():
    # Declare arguments
    description_file = LaunchConfiguration("description_file", default="robot.urdf.xacro")
    prefix = LaunchConfiguration("prefix", default="")
    use_sim_time = LaunchConfiguration('use_sim_time' , default='true')


    rover_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rover_description"), "robots/robot.urdf.xacro"]),
    ])

    rover_description_param = launch_ros.descriptions.ParameterValue(rover_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        #namespace=robot_id,
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': rover_description_param,
          'publish_frequency': 100.0,
          'frame_prefix': prefix,
        }],
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        #namespace=robot_id,
        output='screen',
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)