from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    body_pose_node = Node(
        package='slider_state_estimation',
        executable='body_pose_estimator.py',
        name='body_pose_estimator',
        output='screen'
    )

    imu_node = Node(
        package='slider_state_estimation',
        executable='imu_pub.py',
        name='imu_reader',
        output='screen'
    )

    return LaunchDescription([
        body_pose_node,
        imu_node
    ])