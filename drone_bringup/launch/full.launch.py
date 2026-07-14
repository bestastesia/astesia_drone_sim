# full.launch.py — 全系统：动力学 + 控制器 + 地图 + 规划器 + RViz
# 控制器订阅 /drone/safe_goal（由 planner 发布），不再 remap 到 /drone/goal
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(package='drone_dynamics', executable='dynamics_node',
             name='drone_dynamics', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('drone_dynamics'),
                           'config', 'dynamics.yaml'])]),
        Node(package='drone_controller', executable='controller_node',
             name='drone_controller', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('drone_controller'),
                           'config', 'controller.yaml'])]),
        Node(package='drone_map', executable='map_node',
             name='drone_map', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('drone_map'),
                           'config', 'map.yaml'])]),
        Node(package='drone_planner', executable='planner_node',
             name='drone_planner', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('drone_planner'),
                           'config', 'planner.yaml'])]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('drone_bringup'),
                           'rviz', 'drone.rviz'])]),
    ])