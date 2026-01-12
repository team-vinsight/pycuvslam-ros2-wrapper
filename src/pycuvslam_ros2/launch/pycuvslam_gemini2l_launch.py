import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_pycuvslam = get_package_share_directory('pycuvslam_ros2')
    pkg_orbbec = get_package_share_directory('orbbec_camera')

    # Path to Orbbec launch file
    orbbec_launch_path = PathJoinSubstitution(
        [pkg_orbbec, 'launch', 'gemini2L.launch.py']
    )

    # Config for pycuvslam
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_pycuvslam, 'config', 'gemini2l_params.yaml'),
        description='Path to camera configuration file'
    )

    # Include Orbbec Launch
    # We set align_mode to HW to ensure depth is registered to color on the device.
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orbbec_launch_path),
        launch_arguments={
            'camera_name': 'camera',
            'depth_registration': 'true', 
            'align_mode': 'HW', 
            'enable_frame_sync': 'true'
        }.items()
    )
    
    # pycuvslam Node
    pycuvslam_node = Node(
        package='pycuvslam_ros2',
        executable='rgbd',
        name='pycuvslam_gemini2l',
        output='screen',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            # Map default Orbbec topics to pycuvslam topics
            {'rgb_topic': '/camera/color/image_raw'},
            {'depth_topic': '/camera/depth/image_raw'},
            {'sys_path_pycuvslam': '/home/intellisense05/pycuvslam/bin/x86_64'},
            {'use_gpu': True},
            # Gemini 2L usually provides depth in mm (uint16). 1000 mm = 1 m.
            {'depth_scale_factor': 1000.0}
        ]
    )

    # Rviz
    rviz_config = os.path.join(pkg_pycuvslam, 'config', 'view_rgbd.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        orbbec_launch,
        pycuvslam_node,
        rviz_node
    ])
