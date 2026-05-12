"""Visual odometry on EuRoC — XFeat + LighterGlue + PLNet wireframe (lines).

Same shape as vo_euroc_xfeat_lighterglue.launch.py but points at the lines
config (line_extractor=1, mapline publisher on).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('air_slam_xfeat')

    config_path = LaunchConfiguration('config_path')
    dataroot = LaunchConfiguration('dataroot')
    camera_config_path = LaunchConfiguration('camera_config_path')
    model_dir = LaunchConfiguration('model_dir')
    saving_dir = LaunchConfiguration('saving_dir')
    visualization = LaunchConfiguration('visualization')
    max_frames = LaunchConfiguration('max_frames')
    skip_save_map = LaunchConfiguration('skip_save_map')

    return LaunchDescription([
        DeclareLaunchArgument('config_path',
            default_value=os.path.join(pkg_share, 'configs', 'visual_odometry', 'vo_euroc_xfeat_lighterglue_lines.yaml')),
        DeclareLaunchArgument('dataroot', default_value='/home/maikel/datasets/euroc/MH_03_medium/mav0'),
        DeclareLaunchArgument('camera_config_path',
            default_value=os.path.join(pkg_share, 'configs', 'camera', 'euroc.yaml')),
        DeclareLaunchArgument('model_dir', default_value=os.path.join(pkg_share, 'output')),
        DeclareLaunchArgument('saving_dir', default_value='/tmp/airslam_xfeat_lines'),
        DeclareLaunchArgument('visualization', default_value='true'),
        DeclareLaunchArgument('max_frames', default_value='0'),
        DeclareLaunchArgument('skip_save_map', default_value='0'),
        Node(
            package='air_slam_xfeat', executable='visual_odometry',
            name='visual_odometry', output='screen',
            parameters=[{'config_path': config_path, 'dataroot': dataroot,
                         'camera_config_path': camera_config_path,
                         'model_dir': model_dir, 'saving_dir': saving_dir,
                         'max_frames': max_frames,
                         'skip_save_map': skip_save_map}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'vo_jazzy.rviz')],
            output='screen', condition=IfCondition(visualization),
        ),
    ])
