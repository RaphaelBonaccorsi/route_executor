#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher that will publish the waypoint index to the topic
        self.publisher_ = self.create_publisher(Int32, '/drone/waypoint_index', 10)

        # Waypoint index counter
        self.waypoint_index = 0

        # Publish every 20 seconds (20,000 milliseconds)
        self.timer_period = 20.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_waypoint_index)

        self.get_logger().info("Waypoint publisher started, publishing every 20 seconds...")

    def publish_waypoint_index(self):
        """
        Publish the current waypoint index and increment it.
        """
        msg = Int32()
        msg.data = self.waypoint_index
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published waypoint index: {self.waypoint_index}")

        # Increment the waypoint index for the next cycle
        self.waypoint_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
