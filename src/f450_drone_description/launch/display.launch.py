from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

use_gui = DeclareLaunchArgument('use_gui', default_value='false')
xacro_file = DeclareLaunchArgument('xacro_file', default_value='f450.urdf.xacro')

def generate_launch_description():

    pkg_share = get_package_share_directory('f450_drone_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('xacro_file')])
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'f450_display.rviz')

    # Proper: xacro -> string using Command, then wrap as ParameterValue(str)
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Switch GUI on/off
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui')),  # shown when use_gui==true (handled by default arg)
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([use_gui, xacro_file, rsp, jsp_gui, rviz])
