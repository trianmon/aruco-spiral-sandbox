from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hover_max_velocity',
            default_value='2.0',
            description='Maximum velocity for hover movement (m/s)'
        ),
        
        DeclareLaunchArgument(
            'hover_max_acceleration',
            default_value='1.0',
            description='Maximum acceleration for hover movement (m/s²)'
        ),
        
        DeclareLaunchArgument(
            'hover_smoothing_factor',
            default_value='0.15',
            description='Smoothing factor for movement (0.0-1.0)'
        ),
        
        DeclareLaunchArgument(
            'hover_position_tolerance',
            default_value='0.5',
            description='Position tolerance for movement (m)'
        ),
        
        DeclareLaunchArgument(
            'hover_relative_height',
            default_value='3.0',
            description='Height above marker for hover (m)'
        ),

        Node(
            package='aruco_detector_cpp',
            executable='detector_node',
            name='aruco_detector',
            output='screen',
            parameters=[]
        ),
        
        Node(
            package='hover_marker_mode',
            executable='hover_mode_node',
            name='hover_marker_mode',
            output='screen',
            parameters=[{
                'hover_marker.max_velocity': LaunchConfiguration('hover_max_velocity'),
                'hover_marker.max_acceleration': LaunchConfiguration('hover_max_acceleration'),
                'hover_marker.smoothing_factor': LaunchConfiguration('hover_smoothing_factor'),
                'hover_marker.position_tolerance': LaunchConfiguration('hover_position_tolerance'),
                'hover_marker.relative_height': LaunchConfiguration('hover_relative_height'),
            }]
        ),
    ])
