# full.launch.py — 全系统：动力学+控制器+地图+规划器+RViz
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dyn_yaml  = os.path.join(get_package_share_directory('drone_dynamics'),  'config', 'dynamics.yaml')
    ctrl_yaml = os.path.join(get_package_share_directory('drone_controller'), 'config', 'controller.yaml')
    map_yaml  = os.path.join(get_package_share_directory('drone_map'),        'config', 'map.yaml')
    plan_yaml = os.path.join(get_package_share_directory('drone_planner'),    'config', 'planner.yaml')
    rviz_cfg  = os.path.join(get_package_share_directory('drone_bringup'),    'rviz', 'drone.rviz')

    return LaunchDescription([
        Node(package='drone_dynamics',  executable='dynamics_node',  name='drone_dynamics',
             output='screen', parameters=[dyn_yaml]),
        Node(package='drone_controller', executable='controller_node', name='drone_controller',
             output='screen', parameters=[ctrl_yaml]),
        Node(package='drone_map',        executable='map_node',        name='drone_map',
             output='screen', parameters=[map_yaml]),
        Node(package='drone_planner',    executable='planner_node',    name='drone_planner',
             output='screen', parameters=[plan_yaml]),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg]),
    ])