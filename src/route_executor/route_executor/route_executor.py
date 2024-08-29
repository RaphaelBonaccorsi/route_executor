#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

class RouteExecutor(Node):
    def __init__(self):
        super().__init__('route_executor')

        # Create a publisher to the /mavros/setpoint_position/local topic for position control
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Create service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to be available
        self.get_logger().info('Waiting for arming and set mode services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Get package share directory and set paths to data files
        package_share = get_package_share_directory('route_executor')
        x_file = os.path.join(package_share, 'data', 'x_coords.txt')
        y_file = os.path.join(package_share, 'data', 'y_coords.txt')

        # Read the waypoints from the files
        self.route = self.read_route(x_file, y_file)
        self.current_step = 0

        # Start publishing continuous setpoints before switching to Offboard mode
        self.start_publishing_setpoints()

        # Switch to Offboard mode and arm the drone
        self.set_offboard_mode()
        self.arm()

        # Timer to control the waypoint following (every second, change position based on waypoints)
        self.waypoint_timer = self.create_timer(5.0, self.follow_route)  # Adjust time for moving between waypoints

    def start_publishing_setpoints(self):
        """
        Start publishing setpoints at 20 Hz to ensure PX4 receives valid setpoints.
        """
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        self.get_logger().info('Started publishing setpoints at 20Hz...')

    def set_offboard_mode(self):
        """
        Sets Offboard mode before arming the drone.
        """
        self.get_logger().info('Setting mode to OFFBOARD')
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        set_mode_future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, set_mode_future)

        if set_mode_future.result() is not None and set_mode_future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
        else:
            self.get_logger().error('Failed to set Offboard mode')
            return

    def arm(self):
        """
        Arms the drone after switching to Offboard mode.
        """
        self.get_logger().info('Arming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, arm_future)

        if arm_future.result() is not None and arm_future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm the drone')

    def read_route(self, x_file, y_file):
        """
        Reads x and y coordinates from files and returns a list of tuples (x, y).
        """
        with open(x_file, 'r') as xf, open(y_file, 'r') as yf:
            x_vals = [float(val.strip()) for val in xf.read().split(",")]
            y_vals = [float(val.strip()) for val in yf.read().split(",")]

        # Combine the values into a list of tuples (x, y)
        route = list(zip(x_vals, y_vals))
        return route

    def publish_current_setpoint(self):
        """
        Publishes the current position setpoint to the drone at a high rate.
        This ensures continuous communication in Offboard mode.
        """
        if self.current_step < len(self.route):
            x, y = self.route[self.current_step]

            # Create and publish the position message (setpoint)
            msg = PoseStamped()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 10.0  # Constant altitude of 10 meters
            msg.pose.orientation.w = 1.0  # Neutral orientation (no rotation)
            self.publisher_.publish(msg)

            self.get_logger().info(f"Publishing: Moving to ({x}, {y}, 10.0)")

        else:
            self.get_logger().info("Route complete. Stopping drone.")
            self.setpoint_timer.cancel()  # Stop the setpoint timer
            self.waypoint_timer.cancel()  # Stop the waypoint timer
            self.get_logger().info('Shutting down...')
            rclpy.shutdown()  # Gracefully shut down

    def follow_route(self):
        """
        Move to the next waypoint on the route.
        """
        if self.current_step < len(self.route):
            x, y = self.route[self.current_step]
            self.get_logger().info(f"Moving to waypoint {self.current_step}: ({x}, {y}, 10.0)")
            self.current_step += 1
        else:
            self.get_logger().info("All waypoints visited.")

def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
