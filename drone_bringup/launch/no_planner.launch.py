# no_planner.launch.py — 基础闭环：动力学 + 控制器 + 地图 + RViz（无规划器）
# 启动前自动清理僵尸进程，防止多次 Ctrl+C 后进程叠加
import os, subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def _pre_cleanup():
    for name in ['dynamics_node', 'controller_node', 'map_node',
                 'planner_node', 'rviz2', 'live_monitor']:
        try:
            subprocess.run(['pkill', '-9', '-f', name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
    try:
        subprocess.run(['fuser', '-k', '8765/tcp'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass

def _launch_setup(context, *args, **kwargs):
    _pre_cleanup()
    return []

def generate_launch_description():
    dyn_yaml  = os.path.join(get_package_share_directory('drone_dynamics'),  'config', 'dynamics.yaml')
    ctrl_yaml = os.path.join(get_package_share_directory('drone_controller'), 'config', 'controller.yaml')
    map_yaml  = os.path.join(get_package_share_directory('drone_map'),        'config', 'map.yaml')
    rviz_cfg  = os.path.join(get_package_share_directory('drone_bringup'),    'rviz', 'drone.rviz')

    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
        Node(package='drone_dynamics',  executable='dynamics_node',  name='drone_dynamics',
             output='screen', parameters=[dyn_yaml]),
        Node(package='drone_controller', executable='controller_node', name='drone_controller',
             output='screen', parameters=[ctrl_yaml],
             remappings=[('/drone/safe_goal', '/drone/goal')]),
        Node(package='drone_map',        executable='map_node',        name='drone_map',
             output='screen', parameters=[map_yaml]),
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_cfg]),
    ])