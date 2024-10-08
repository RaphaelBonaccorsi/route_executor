#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty
from ament_index_python.packages import get_package_share_directory
from plansys2_msgs.srv import GetPlan, AddProblem, GetDomain
from plansys2_msgs.msg import ActionExecution

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

        self.domain_file = os.path.join('pddl', 'domain.pddl')
        self.problem_file = os.path.join('pddl', 'problem.pddl')
        # Load the domain and problem files (PDDL)
        self.load_pddl_files()
        """
        # Create a PlanSys2 plan service client
        self.plan_client = self.create_client(GetPlan, '/planner/get_plan')

        # Generate the plan
        self.get_logger().info("Generating the plan...")
        self.plan = self.generate_plan()

        # Timer to publish the waypoint index every 20 seconds
        self.timer_period = 20.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.execute_plan_step)
        """
    def load_pddl_files(self):
        """
        Load the PDDL domain and problem files into PlanSys2.
        """
        self.get_logger().info(f"Loaded PDDL files: {self.domain_file}, {self.problem_file}")

    def generate_plan(self):
        """
        Generate a plan based on the PDDL problem.
        """

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

class DomainExpertClient(Node):
    def __init__(self):
        super().__init__('domain_expert_client')
        # Cria o cliente para o serviço GetDomain
        self.get_domain_client = self.create_client(GetDomain, '/domain_expert/get_domain')
        # Espera pelo serviço estar disponível
        self.get_domain_client.wait_for_service(timeout_sec=10.0)

    def get_domain(self):
        # Cria um pedido vazio, pois o serviço não necessita de argumentos
        request = GetDomain.Request()
        
        # Envia o pedido e espera pela resposta
        future = self.get_domain_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Verifica se a chamada foi bem-sucedida
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Domínio recebido: {response.domain}")
        else:
            self.get_logger().error("Falha ao chamar o serviço /domain_expert/get_domain")

def main(args=None):
    rclpy.init(args=args)
    domain_client = DomainExpertClient()
    domain_client.get_domain()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
