#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.create_subscription(Path, 'planned_path', self.path_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.waypoints = []
        self.current_idx = 0
        self.Kp_lin = 0.5
        self.Kp_ang = 1.0

    def path_cb(self, msg):
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_idx = 0

    def odom_cb(self, msg):
        if not self.waypoints: return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        tx, ty = self.waypoints[self.current_idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        err_ang = self.angle_diff(angle_to_goal, yaw)
        cmd = Twist()
        cmd.linear.x = self.Kp_lin * dist
        cmd.angular.z = self.Kp_ang * err_ang
        cmd.linear.x = max(min(cmd.linear.x, 0.5), -0.5)
        cmd.angular.z = max(min(cmd.angular.z, 1.0), -1.0)
        self.pub.publish(cmd)
        if dist < 0.2 and self.current_idx < len(self.waypoints) - 1:
            self.current_idx += 1

    @staticmethod
    def quat_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def angle_diff(target, current):
        d = target - current
        while d > math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

def main():
    rclpy.init()
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
