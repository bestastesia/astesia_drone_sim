#!/usr/bin/env python3
"""
waypoint_sender.py — 按顺序发布多目标点（正方形航线等）
用法:
  ros2 run drone_bringup waypoint_sender.py
  ros2 run drone_bringup waypoint_sender.py --ros-args -p waypoints:="[[0,0,1.5],[2,0,1.5],[2,2,1.5],[0,2,1.5],[0,0,1.5]]" -p hold_time:=5.0
"""
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        default_wps = "[[0.0,0.0,1.5],[2.0,0.0,1.5],[2.0,2.0,1.5],[0.0,2.0,1.5],[0.0,0.0,1.5]]"
        self.declare_parameter('waypoints', default_wps)
        self.declare_parameter('hold_time', 5.0)

        raw = self.get_parameter('waypoints').get_parameter_value().string_value
        self.waypoints_ = json.loads(raw)

        self.hold_time_ = self.get_parameter('hold_time').get_parameter_value().double_value
        self.pub_ = self.create_publisher(PoseStamped, '/drone/goal', 10)
        self.idx_ = 0
        self.at_wp_since_ = None
        self.timer_ = self.create_timer(0.2, self.tick)
        self.get_logger().info(f'waypoint_sender: {len(self.waypoints_)} waypoints, hold={self.hold_time_}s')

    def tick(self):
        if self.idx_ >= len(self.waypoints_):
            self.get_logger().info('all waypoints complete — holding last')
            return
        wp = self.waypoints_[self.idx_]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(wp[0])
        msg.pose.position.y = float(wp[1])
        msg.pose.position.z = float(wp[2])
        self.pub_.publish(msg)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.at_wp_since_ is None:
            self.at_wp_since_ = now
            self.get_logger().info(f'-> waypoint {self.idx_+1}/{len(self.waypoints_)}: ({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f})')
        if now - self.at_wp_since_ >= self.hold_time_:
            self.idx_ += 1
            self.at_wp_since_ = None
            if self.idx_ < len(self.waypoints_):
                nw = self.waypoints_[self.idx_]
                self.get_logger().info(f'advancing to wp {self.idx_+1}: ({nw[0]:.1f},{nw[1]:.1f},{nw[2]:.1f})')

def main():
    rclpy.init()
    rclpy.spin(WaypointSender())
    rclpy.shutdown()

if __name__ == '__main__':
    main()