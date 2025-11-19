from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    frame = DeclareLaunchArgument('frame', default_value='map')
    
    # CORRECTED: Listen-only mode for ESP32 WiFi bridge
    fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@',  # ← FIXED!
        description='MAVLink URL for ESP32 WiFi bridge'
    )

    show_url = LogInfo(msg=['[mavros] fcu_url = ', LaunchConfiguration('fcu_url')])

    mavros = Node(
        package='mavros', 
        executable='mavros_node', 
        output='screen',
        parameters=[{'fcu_url': LaunchConfiguration('fcu_url')}]
    )

    path_node = Node(
        package='f450_drone_bridge', 
        executable='path_node', 
        output='screen',
        parameters=[{'frame_id': LaunchConfiguration('frame')}]
    )

    cardinal_markers = Node(
        package='f450_drone_bridge',
        executable='cardinal_markers_node',
        output='screen',
        name='cardinal_markers',
        parameters=[
            {'frame_id': 'map'},
            {'distance': 1.5},
            {'text_scale': 0.2},
            {'text_height': 0.0}
        ]
    )

    # Attitude to TF publisher (C++)
    attitude_tf = Node(
        package='f450_drone_bridge',
        executable='attitude_to_tf_node',
        output='screen',
        name='attitude_to_tf',
        parameters=[
            {'parent_frame': 'map'},
            {'child_frame': 'base_link'}
        ]
    )

    # imu node corrector for attitude plugin (C++)
    imu_roll_corrector = Node(
        package='f450_drone_bridge',
        executable='imu_roll_corrector_node',
        output='screen',
        name='imu_roll_corrector',
        parameters=[
        ]
    )

    return LaunchDescription([
        frame,
        fcu_url, 
        show_url,
        mavros,
        cardinal_markers,
        attitude_tf,
        imu_roll_corrector
        #path_node
    ])