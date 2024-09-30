#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.srv import GetPlan
from plansys2_msgs.msg import ActionExecution

from plansys2_problem_expert.problem_expert import ProblemExpertClient
from plansys2_domain_expert.domain_expert import DomainExpertClient

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')

        # Publisher to publish the waypoint index to the topic
        self.publisher_ = self.create_publisher(Int32, '/drone/waypoint_index', 10)

        # Waypoint index counter
        self.waypoint_index = 0

        # PlanSys2 clients to interact with the domain and problem
        self.problem_client = ProblemExpertClient()
        self.domain_client = DomainExpertClient()

        # PDDL files
        package_share = get_package_share_directory('pddl_reader')
        self.domain_file = os.path.join(package_share, 'data', 'domain.pddl')
        self.problem_file = os.path.join(package_share, 'data', 'problem.pddl')

        # Load the domain and problem files (PDDL)
        self.load_pddl_files()

        # Create a PlanSys2 plan service client
        self.plan_client = self.create_client(GetPlan, '/planner/get_plan')

        # Generate the plan
        self.get_logger().info("Generating the plan...")
        self.plan = self.generate_plan()

        # Timer to publish the waypoint index every 20 seconds
        self.timer_period = 20.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.execute_plan_step)

    def load_pddl_files(self):
        """
        Load the PDDL domain and problem files into PlanSys2.
        """
        self.get_logger().info(f"Loaded PDDL files: {self.domain_file}, {self.problem_file}")

    def generate_plan(self):
        """
        Generate a plan based on the PDDL problem.
        """
        # Define the goal (this should be consistent with the PDDL goal)
        goal = self.problem_client.parse_goal("(and (drone_at drone1 waypoint_5))")
        self.problem_client.set_goal(goal)

        # Get the plan
        self.get_logger().info("Requesting plan from PlanSys2...")
        request = GetPlan.Request()
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and len(future.result().plan.items) > 0:
            plan = future.result().plan.items
            self.get_logger().info(f"Plan generated successfully with {len(plan)} steps.")
            return plan
        else:
            self.get_logger().error("Failed to generate plan or plan is empty.")
            return None

    def execute_plan_step(self):
        """
        Execute a step in the plan by sending the drone to the next waypoint.
        """
        if self.plan is None:
            self.get_logger().error("No plan available to execute.")
            return

        if self.waypoint_index < len(self.plan):
            action = self.plan[self.waypoint_index]
            self.get_logger().info(f"Executing action: {action.action}")

            # Extract the waypoint index from the plan action (assuming action format: (move drone waypoint_X waypoint_Y))
            waypoint_name = action.arguments[2]
            waypoint_idx = int(waypoint_name.replace("waypoint_", ""))

            # Publish the waypoint index to the drone
            msg = Int32()
            msg.data = waypoint_idx
            self.publisher_.publish(msg)

            self.get_logger().info(f"Published waypoint index: {waypoint_idx}")

            # Increment the index for the next action
            self.waypoint_index += 1
        else:
            self.get_logger().info("Plan execution complete.")
            self.timer.cancel()  # Stop the timer when the plan is complete

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
