#!/usr/bin/env python3
"""
trajectory_generator.py — 参数化轨迹生成器 (rclpy)
用法:
  ros2 run drone_bringup trajectory_generator.py --ros-args -p type:=circle
  ros2 run drone_bringup trajectory_generator.py --ros-args -p type:=lemniscate -p period:=15.0

轨迹类型:
  circle      — 水平圆轨迹，半径 radius，圆心 (cx, cy, cz)
  lemniscate  — 水平八字轨迹 (Bernoulli lemniscate)，尺度 scale
  waypoints   — 从 YAML/JSON 文件读取航点列表，逐点停留 hold_time 秒后切换
"""
import sys, math, time, json, os, threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # --- 轨迹参数 ---
        self.declare_parameter('type', 'circle')
        self.declare_parameter('rate', 10.0)  # Hz
        self.declare_parameter('period', 10.0)  # 一圈用时 (s)
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('scale', 1.5)  # lemniscate 尺度
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)
        self.declare_parameter('cz', 1.5)
        self.declare_parameter('hold_time', 3.0)  # waypoint 停留时间
        self.declare_parameter('waypoint_file', '')  # waypoints.json 路径

        self.goal_pub = self.create_publisher(PoseStamped, '/drone/goal', 10)

        self._read_params()

        # 定时器——按 rate Hz 发布 next goal
        dt = 1.0 / max(self.rate, 1.0)
        self.timer = self.create_timer(dt, self._tick)

        self.t0 = time.time()
        self.wp_idx = 0      # waypoint 模式索引
        self.wp_start = 0.0  # 当前航点开始时间
        self.get_logger().info(
            f'trajectory generator running: type={self.traj_type} rate={self.rate}Hz')

    # ------------------------------------------------------------------
    def _read_params(self):
        self.traj_type = self.get_parameter('type').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.period = self.get_parameter('period').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value
        self.cz = self.get_parameter('cz').get_parameter_value().double_value
        self.hold_time = self.get_parameter('hold_time').get_parameter_value().double_value
        self.wp_file = self.get_parameter('waypoint_file').get_parameter_value().string_value

        # waypoints 文件
        self.waypoints = []
        if self.traj_type == 'waypoints':
            if self.wp_file and os.path.exists(self.wp_file):
                with open(self.wp_file) as f:
                    self.waypoints = json.load(f)
                self.get_logger().info(f'loaded {len(self.waypoints)} waypoints from {self.wp_file}')
            else:
                # 默认正方形航点
                self.waypoints = [
                    [0.0, 0.0, 1.5], [2.0, 0.0, 1.5],
                    [2.0, 2.0, 1.5], [0.0, 2.0, 1.5], [0.0, 0.0, 1.5],
                ]
            self.wp_idx = 0
            self.wp_start = time.time()
            self._pub_goal(*self.waypoints[0])

    # ------------------------------------------------------------------
    def _tick(self):
        t = time.time() - self.t0

        if self.traj_type == 'circle':
            x, y, z = self._circle(t)
        elif self.traj_type == 'lemniscate':
            x, y, z = self._lemniscate(t)
        elif self.traj_type == 'waypoints':
            self._waypoint_advance(t)
            return   # _waypoint_advance 内部 pub
        else:
            self.get_logger().warn(f'unknown trajectory type: {self.traj_type}')
            return

        self._pub_goal(x, y, z)

    # ------------------------------------------------------------------
    # 轨迹数学
    # ------------------------------------------------------------------
    def _circle(self, t):
        omega = 2.0 * math.pi / self.period
        x = self.cx + self.radius * math.cos(omega * t)
        y = self.cy + self.radius * math.sin(omega * t)
        z = self.cz
        return x, y, z

    def _lemniscate(self, t):
        """Bernoulli lemniscate: r² = a² cos(2θ), 参数化用角度扫过"""
        omega = 2.0 * math.pi / self.period
        theta = omega * t
        # 参数表示: (a cosθ / (1+sin²θ), a sinθ cosθ / (1+sin²θ))
        denom = 1.0 + math.sin(theta)**2
        x = self.cx + self.scale * math.cos(theta) / denom
        y = self.cy + self.scale * math.sin(theta) * math.cos(theta) / denom
        z = self.cz
        return x, y, z

    def _waypoint_advance(self, t):
        elapsed = t - self.wp_start
        if elapsed >= self.hold_time:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            self.wp_start = t
            wp = self.waypoints[self.wp_idx]
            self._pub_goal(*wp)
            self.get_logger().info(
                f'waypoint {self.wp_idx+1}/{len(self.waypoints)}: ({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f})')

    # ------------------------------------------------------------------
    def _pub_goal(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
