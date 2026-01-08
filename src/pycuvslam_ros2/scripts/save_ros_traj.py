#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class TrajSaver(Node):
    def __init__(self):
        super().__init__('traj_saver')
        self.output_file = 'ros_traj.txt'
        self.f = open(self.output_file, 'w')
        self.f.write("# timestamp tx ty tz qx qy qz qw\n")
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.get_logger().info(f"Saving trajectory to {self.output_file}...")

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(f"{timestamp} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n")
        self.f.flush() # Ensure data is written

    def destroy_node(self):
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    traj_saver = TrajSaver()
    try:
        rclpy.spin(traj_saver)
    except KeyboardInterrupt:
        pass
    finally:
        traj_saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
