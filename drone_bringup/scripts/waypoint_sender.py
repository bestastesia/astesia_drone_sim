#!/usr/bin/env python3
"""
waypoint_sender.py — 按顺序发布多目标点（不依赖 rclpy，绕过 conda Python 冲突）
用法:
  ros2 run drone_bringup waypoint_sender.py
  ros2 run drone_bringup waypoint_sender.py --ros-args -p waypoints:="[[0,0,1.5],[2,0,1.5]]" -p hold_time:=4.0

注意: 脚本内部用 subprocess 调 ros2 topic pub，因此需要 ros2 在 PATH。
"""
import json
import subprocess
import sys
import time
import argparse

DEFAULT_WAYPOINTS = "[[0.0,0.0,1.5],[2.0,0.0,1.5],[2.0,2.0,1.5],[0.0,2.0,1.5],[0.0,0.0,1.5]]"

def pub_goal(x, y, z):
    """通过 ros2 topic pub 发布一个 PoseStamped"""
    cmd = [
        'ros2', 'topic', 'pub', '--once',
        '/drone/goal', 'geometry_msgs/msg/PoseStamped',
        f'{{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}}}'
    ]
    subprocess.run(cmd, capture_output=True)

def parse_ros_args():
    """简单解析 --ros-args -p key:=value 来提取 hold_time 和 waypoints"""
    hold = 5.0
    wps_str = DEFAULT_WAYPOINTS
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == '--ros-args':
            i += 1
            continue
        if args[i] == '-p' and i + 1 < len(args):
            kv = args[i + 1]
            if 'hold_time:=' in kv:
                hold = float(kv.split(':=')[1])
            elif 'waypoints:=' in kv:
                wps_str = kv.split(':=', 1)[1]
            elif ':=' not in kv:
                # 可能是 waypoints 直接写
                if kv.startswith('[['):
                    wps_str = kv
            i += 2
            continue
        i += 1
    return hold, json.loads(wps_str)

def main():
    hold_time, waypoints = parse_ros_args()
    print(f"waypoint_sender: {len(waypoints)} waypoints, hold={hold_time}s", flush=True)

    for idx, wp in enumerate(waypoints):
        x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
        print(f"-> waypoint {idx+1}/{len(waypoints)}: ({x:.1f}, {y:.1f}, {z:.1f})", flush=True)
        # 在停留时间内持续发送，确保控制器收到
        start = time.time()
        while time.time() - start < hold_time:
            pub_goal(x, y, z)
            time.sleep(0.5)
    print("all waypoints complete", flush=True)

if __name__ == '__main__':
    main()