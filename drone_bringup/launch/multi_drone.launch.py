# multi_drone.launch.py — 2 架无人机，命名空间隔离
# 启动前自动清理僵尸进程，防止多次 Ctrl+C 后进程叠加
import os, subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory

def _pre_cleanup():
    for name in ['dynamics_node', 'controller_node', 'map_node',
                 'planner_node', 'rviz2', 'live_monitor']:
        try:
            subprocess.run(['pkill', '-9', '-f', name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception: pass
    try:
        subprocess.run(['fuser', '-k', '8765/tcp'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception: pass

def _launch_setup(context, *args, **kwargs):
    _pre_cleanup()
    return []

def generate_launch_description():
    dyn_yaml  = os.path.join(get_package_share_directory('drone_dynamics'),  'config', 'dynamics.yaml')
    ctrl_yaml = os.path.join(get_package_share_directory('drone_controller'), 'config', 'controller.yaml')
    map_yaml  = os.path.join(get_package_share_directory('drone_map'),        'config', 'map.yaml')
    rviz_cfg  = os.path.join(get_package_share_directory('drone_bringup'),    'rviz', 'drone.rviz')

    def drone_ns(ns, init_pose, init_yaw=0.0):
        """Create a namespaced drone with its own dynamics + controller"""
        dyn_params = [dyn_yaml, {'init_pose': init_pose}]
        return GroupAction([
            PushRosNamespace(ns),
            Node(package='drone_dynamics',  executable='dynamics_node',
                 name='drone_dynamics', output='screen', parameters=dyn_params),
            Node(package='drone_controller', executable='controller_node',
                 name='drone_controller', output='screen', parameters=[ctrl_yaml],
                 remappings=[('drone/safe_goal', 'drone/goal')]),
        ])

    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
        # Shared nodes (global namespace)
        Node(package='drone_map', executable='map_node', name='drone_map',
             output='screen', parameters=[map_yaml]),
        # Drone 0: origin hover
        drone_ns('drone0', init_pose=[0.0, 0.0, 0.0, 0.0]),
        # Drone 1: offset 1.5m to the right
        drone_ns('drone1', init_pose=[1.5, 0.0, 0.0, 0.0]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_cfg]),
    ])
