#!/usr/bin/env python3
import os
import json
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Int32
import math
from math import sqrt


class CoordinateConverter:
    """
    A utility class for converting geographic coordinates (latitude, longitude)
    to local Cartesian ENU (East-North-Up) coordinates.

    Methods
    -------
    geo_to_cart(geo_point, geo_home):
        Converts geographic coordinates to ENU coordinates relative to a home position.
    """

    @staticmethod
    def geo_to_cart(geo_point, geo_home):
        """
        Convert geographic coordinates (latitude, longitude) to local ENU (X, Y) coordinates
        relative to the home position.

        Parameters
        ----------
        geo_point : tuple
            A tuple containing the latitude and longitude of the target point (lat, lon).
        geo_home : tuple
            A tuple containing the latitude and longitude of the home position (lat, lon).

        Returns
        -------
        tuple
            The ENU coordinates (X, Y) relative to the home position in meters.
        """
        def calc_y(lat, home_lat):
            return (lat - home_lat) * 111320.0

        def calc_x(longi, home_long, home_lat):
            return (longi - home_long) * (111320.0 * math.cos(home_lat * math.pi / 180))

        x = calc_x(geo_point[1], geo_home[1], geo_home[0])  # Longitude
        y = calc_y(geo_point[0], geo_home[0])  # Latitude

        return x, y


class Map:
    """
    Handles loading waypoints from a JSON file and converting them to ENU coordinates.

    Attributes
    ----------
    home_lat : float
        Latitude of the home position.
    home_lon : float
        Longitude of the home position.
    converter : CoordinateConverter
        Instance of the CoordinateConverter class for coordinate conversion.

    Methods
    -------
    read_route_from_json(map_file):
        Reads the centers of ROIs from a JSON file and converts them to ENU coordinates.
    """

    def __init__(self, home_lat, home_lon):
        """
        Initializes the Map class with a home position.

        Parameters
        ----------
        home_lat : float
            The latitude of the home position.
        home_lon : float
            The longitude of the home position.
        """
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.converter = CoordinateConverter()

    def read_route_from_json(self, map_file):
        """
        Reads the centers of regions of interest (ROIs) from a JSON file and converts them
        from latitude/longitude to ENU coordinates relative to the home position.

        Parameters
        ----------
        map_file : str
            Path to the JSON file containing the route information.

        Returns
        -------
        list of tuple
            A list of tuples containing the ENU coordinates (X, Y) of each waypoint.
        """
        with open(map_file, 'r') as file:
            data = json.load(file)

        route = []
        for roi in data.get('roi', []):
            center = roi.get('center')
            if center:
                enu_x, enu_y = self.converter.geo_to_cart((center[1], center[0]), (self.home_lat, self.home_lon))
                route.append((enu_x, enu_y))
        return route


class RouteExecutor(Node):
    """
    Manages drone control, waypoint navigation, and interaction with MAVROS.

    Attributes
    ----------
    home_lat : float
        Latitude of the home position.
    home_lon : float
        Longitude of the home position.
    home_alt : float
        Altitude of the home position.
    map : Map
        Instance of the Map class for handling waypoints.
    publisher_ : rclpy.Publisher
        ROS 2 publisher to publish setpoints to the drone.
    arming_client : rclpy.Client
        ROS 2 client for the arming service.
    set_mode_client : rclpy.Client
        ROS 2 client for setting the flight mode.
    waypoint_subscriber : rclpy.Subscription
        ROS 2 subscriber to receive waypoint indices.
    route : list of tuple
        List of waypoints as ENU coordinates.
    current_waypoint_index : int or None
        The index of the current waypoint being navigated to.
    waypoints : list of tuple
        List of waypoints to be followed by the drone.

    Methods
    -------
    start_publishing_setpoints():
        Starts publishing setpoints at 20 Hz to PX4.
    set_offboard_mode():
        Sets the drone mode to Offboard before arming.
    arm():
        Arms the drone after switching to Offboard mode.
    publish_current_setpoint():
        Publishes the current setpoint to the drone.
    has_reached_waypoint(x, y, threshold=1.0):
        Checks if the drone has reached the current waypoint within a threshold.
    get_current_position():
        Simulates the current position of the drone (replace with actual position logic).
    waypoint_callback(msg):
        Receives the waypoint index from a topic and updates the current waypoint.
    """

    def __init__(self):
        super().__init__('route_executor')

        self.home_lat = -22.001333
        self.home_lon = -47.934152
        self.home_alt = 0

        # Initialize Map object for handling waypoints
        self.map = Map(self.home_lat, self.home_lon)

        # Create a publisher to the /mavros/setpoint_position/local topic for position control
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Create service clients for arming and setting mode
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to be available
        self.get_logger().info('Waiting for arming and set mode services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Subscriber to listen for the waypoint index published by another node
        self.waypoint_subscriber = self.create_subscription(
            Int32, '/drone/waypoint_index', self.waypoint_callback, 10
        )

        # Get package share directory and set path to JSON map file
        package_share = get_package_share_directory('route_executor')
        map_file = os.path.join(package_share, 'data', 'map.json')

        # Read the waypoints from the JSON file (center of the ROIs)
        self.route = self.map.read_route_from_json(map_file)

        self.current_waypoint_index = None
        self.waypoints = []

        # Start publishing continuous setpoints before switching to Offboard mode
        self.start_publishing_setpoints()

        # Switch to Offboard mode and arm the drone
        time.sleep(10)
        self.set_offboard_mode()
        self.arm()

    def start_publishing_setpoints(self):
        """
        Starts publishing setpoints at 20 Hz to ensure PX4 receives valid setpoints.
        """
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        self.get_logger().info('Started publishing setpoints at 20Hz...')

    def set_offboard_mode(self):
        """
        Sets the drone mode to Offboard before arming the drone.
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

    def publish_current_setpoint(self):
        """
        Publishes the current setpoint (waypoint) to the drone.
        """
        if self.current_waypoint_index is None or len(self.waypoints) == 0:
            return

        if self.current_waypoint_index < len(self.waypoints):
            x, y = self.waypoints[self.current_waypoint_index]

            msg = PoseStamped()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 10.0
            msg.pose.orientation.w = 1.0
            self.publisher_.publish(msg)

            self.get_logger().info(f"Publishing: Moving to ({x}, {y}, 10.0)")

            if self.has_reached_waypoint(x, y):
                self.get_logger().info(f"Reached waypoint ({x}, {y})")
                if self.current_waypoint_index < len(self.waypoints) - 1:
                    self.current_waypoint_index += 1
                else:
                    self.get_logger().info("All waypoints reached.")

    def has_reached_waypoint(self, x, y, threshold=1.0):
        """
        Check if the drone has reached the current waypoint within a given threshold.

        Parameters
        ----------
        x : float
            The X coordinate of the waypoint.
        y : float
            The Y coordinate of the waypoint.
        threshold : float, optional
            The threshold distance in meters to consider the waypoint reached (default is 1.0).

        Returns
        -------
        bool
            True if the drone has reached the waypoint, False otherwise.
        """
        current_position = self.get_current_position()
        distance = sqrt((x - current_position[0]) ** 2 + (y - current_position[1]) ** 2)
        return distance < threshold

    def get_current_position(self):
        """
        Simulates the current position of the drone.
        Replace this with actual logic to get the drone's position from a ROS topic or service.

        Returns
        -------
        tuple
            The current position (X, Y) of the drone.
        """
        return (0, 0)

    def waypoint_callback(self, msg):
        """
        Receives the waypoint index from a topic and updates the current waypoint.

        Parameters
        ----------
        msg : std_msgs.msg.Int32
            The message containing the waypoint index.
        """
        self.get_logger().info(f"Received waypoint index: {msg.data}")
        self.current_waypoint_index = int(msg.data)

        if self.current_waypoint_index < len(self.route):
            self.waypoints.append(self.route[self.current_waypoint_index])
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} stored: {self.route[self.current_waypoint_index]}")


def main(args=None):
    rclpy.init(args=args)
    node = RouteExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
