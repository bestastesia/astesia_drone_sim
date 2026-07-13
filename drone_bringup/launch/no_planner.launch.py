# no_planner.launch.py — 基础闭环：动力学 + 控制器 + RViz
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_dynamics',
            executable='dynamics_node',
            name='drone_dynamics',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('drone_dynamics'), 'config', 'dynamics.yaml'
            ])],
        ),
        Node(
            package='drone_controller',
            executable='controller_node',
            name='drone_controller',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('drone_controller'), 'config', 'controller.yaml'
            ])],
            remappings=[('/drone/safe_goal', '/drone/goal')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('drone_bringup'), 'rviz', 'drone.rviz'
            ])],
        ),
    ])