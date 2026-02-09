from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'goal_x',
            default_value='2.0',
            description='Goal X coordinate'
        ),
        DeclareLaunchArgument(
            'goal_y',
            default_value='2.0',
            description='Goal Y coordinate'
        ),
        
        Node(
            package='custom_dwa_planner',
            executable='dwa_planner',
            name='dwa_planner',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),
        
        Node(
            package='custom_dwa_planner',
            executable='goal_publisher',
            name='goal_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y')
            }]
        ),
    ])
