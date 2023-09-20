import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
     
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        '3d_lidar.rviz')
    
    rviz_node =  Node(package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output={'both': 'log'}
            )
    
     
    setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/robot_state_publisher.launch.py'])
        )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'),
            '/3s_slam_launch.launch.py'])
        )
    

    return LaunchDescription([
        setup,
        rviz_node
    ])