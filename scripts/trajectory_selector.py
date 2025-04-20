#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

class TrajectorySelector(Node):
    def __init__(self):
        super().__init__('trajectory_selector')
        self.pub = self.create_publisher(Path, 'planned_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'

        # Set up matplotlib for click input
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Click to add waypoints; close window when done')
        self.ax.set_xlim(-1, 10); self.ax.set_ylim(-1, 10)
        self.scatter, = self.ax.plot([], [], 'ro-')
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show(block=False)

        # Publish path at 1 Hz
        self.create_timer(1.0, self.publish_path)

    def onclick(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0
        self.path.poses.append(ps)

        xs = [p.pose.position.x for p in self.path.poses]
        ys = [p.pose.position.y for p in self.path.poses]
        self.scatter.set_data(xs, ys)
        self.fig.canvas.draw()

    def publish_path(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path)

def main():
    rclpy.init()
    node = TrajectorySelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
