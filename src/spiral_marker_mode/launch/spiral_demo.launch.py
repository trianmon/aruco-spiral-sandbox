from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'spiral_max_velocity',
            default_value='3.0',
            description='Maximum velocity for spiral movement (m/s)'
        ),
        
        DeclareLaunchArgument(
            'spiral_angular_velocity',
            default_value='0.5',
            description='Angular velocity for spiral (rad/s)'
        ),
        
        DeclareLaunchArgument(
            'spiral_relative_height',
            default_value='10.0',
            description='Height above marker for spiral (m)'
        ),

        Node(
            package='aruco_detector_cpp',
            executable='detector_node',
            name='aruco_detector',
            output='screen',
            parameters=[]
        ),
        
        Node(
            package='spiral_marker_mode',
            executable='spiral_mode_node',
            name='spiral_marker_mode',
            output='screen',
            parameters=[{
                'spiral_marker.max_velocity': LaunchConfiguration('spiral_max_velocity'),
                'spiral_marker.angular_velocity': LaunchConfiguration('spiral_angular_velocity'),
                'spiral_marker.relative_height': LaunchConfiguration('spiral_relative_height'),
                'spiral_marker.max_acceleration': 1.5,
                'spiral_marker.position_tolerance': 0.3,
                'spiral_marker.radius_rate': 0.2,
            }]
        ),
    ])
