from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pycuvslam_ros2'),
        'config',
        'camera_params.yaml'
    )
    
    rviz_config = os.path.join(
        get_package_share_directory('pycuvslam_ros2'),
        'config',
        'view_rgbd.rviz'
    )

    return LaunchDescription([
        Node(
            package='pycuvslam_ros2',
            executable='rgbd',
            name='pycuvslam_rgbd',
            output='screen',
            parameters=[
                {'config_file': config},
                {'rgb_topic': '/camera/rgb/image_raw'},
                {'depth_topic': '/camera/depth/image_raw'},
                {'sys_path_pycuvslam': '/home/intellisense05/pycuvslam/bin/x86_64'},
                {'use_gpu': True}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
