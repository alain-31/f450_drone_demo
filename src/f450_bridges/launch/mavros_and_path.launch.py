from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    frame = DeclareLaunchArgument('frame', default_value='map')
    fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://@0.0.0.0:14550',  # Wi-Fi MAVLink bridge default
        description='MAVLink URL (e.g., udp://@0.0.0.0:14550 or serial:///dev/ttyUSB0:57600)'
    )

    mavros = Node(
        package='mavros', executable='mavros_node', output='screen',
        parameters=[{'fcu_url': LaunchConfiguration('fcu_url')}]
    )

    path_node = Node(
        package='f450_bridges', executable='path_node', output='screen',
        parameters=[{'frame_id': LaunchConfiguration('frame')}]
    )

    return LaunchDescription([frame, fcu_url, mavros, path_node])
