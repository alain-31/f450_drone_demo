from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_gui = DeclareLaunchArgument('use_gui', default_value='true')
    xacro_file = DeclareLaunchArgument('xacro_file', default_value='f450.urdf.xacro')

    pkg_share = get_package_share_directory('f450_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('xacro_file')])
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'f450_display.rviz')

    # xacro -> URDF on the fly
    robot_description = {'robot_description':
        ExecuteProcess(
            cmd=['xacro', urdf_path],
            output='screen'
        )
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen'
    )

    return LaunchDescription([use_gui, xacro_file, rsp, jsp, rviz])
