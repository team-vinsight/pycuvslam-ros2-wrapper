from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
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

    depth_scale_factor_arg = DeclareLaunchArgument(
        'depth_scale_factor',
        default_value='1000.0',
        description='Scale factor for depth values (1000.0 for mm to meters)'
    )

    return LaunchDescription([
        depth_scale_factor_arg,
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
                {'use_gpu': True},
                {'depth_scale_factor': LaunchConfiguration('depth_scale_factor')}
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
