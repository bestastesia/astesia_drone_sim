# no_planner.launch.py — 基础闭环：动力学 + 控制器 + RViz
# 无地图/避障，safe_goal remap 到 /drone/goal
# 用法：ros2 launch drone_bringup no_planner.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    pkg_dynamics = 'drone_dynamics'
    pkg_controller = 'drone_controller'
    pkg_bringup = 'drone_bringup'

    use_rviz = LaunchConfiguration('use_rviz', default='true')
    init_z = LaunchConfiguration('init_z', default='0.0')

    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', '..', 'drone_bringup', 'rviz', 'drone.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('init_z', default_value='0.0'),

        Node(
            package=pkg_dynamics,
            executable='dynamics_node',
            name='drone_dynamics',
            output='screen',
            parameters=[{'init_pose': [0.0, 0.0, LaunchConfiguration('init_z'), 0.0]}],
        ),
        Node(
            package=pkg_controller,
            executable='controller_node',
            name='drone_controller',
            output='screen',
            remappings=[('/drone/safe_goal', '/drone/goal')],
        ),
        # RViz (可选关闭)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=lambda ctx: ctx.launch_configurations['use_rviz'].lower() == 'true',
        ),
    ])