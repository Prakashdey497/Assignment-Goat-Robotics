#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from amr_custom_msg.srv import GoalList
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf_transformations
from rclpy.action import ActionClient

class SimpleDeliveryRobot(Node):
    def __init__(self):
        super().__init__('simple_delivery_robot_node')

        self.get_logger().info("SimpleDeliveryRobot Node Created!")

        # Initialize variables
        self.pending_orders = []
        self.goal_status = None
        self.current_goal = None

        # Service
        self.create_service(GoalList, 'position_list', self.order_callback)

        # Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Predefined Locations
        self.locations = {
            "home": [0.0, 0.0, 0.0],
            "kitchen": [4.0, 0.0, 0.0],
            "table1": [4.0, -4.0, 1.57],
            "table2": [-5.0, 0.0, 0.0],
            "table3": [-5.0, -4.0, 0.0]
        }

        # Timer
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        """ Periodic status check """
        if not self.pending_orders and self.goal_status != 2:
            self.get_logger().info("Waiting for new orders.")
        elif self.goal_status in [4, None]:  # Goal reached or no goal set
            self.process_next_order()

    def process_next_order(self):
        """ Process next goal from pending_orders """
        if self.pending_orders:
            self.current_goal = self.pending_orders.pop(0)
            self.get_logger().info(f"Navigating to: {self.current_goal}")
            self.send_goal(self.current_goal)

    def order_callback(self, request, response):
        """ Handle goal list request """
        if not request.goal_list:
            self.get_logger().error("Received an empty goal list.")
            response.success = False
            response.message = "Error: Received an empty goal list."
            return response

        # Check for duplicate goals
        if len(request.goal_list) != len(set(request.goal_list)):
            self.get_logger().error("Duplicate goal names found.")
            response.success = False
            response.message = "Error: Goal names must be unique."
            return response

        # Validate requested goals
        valid_goals = [goal for goal in request.goal_list if goal in self.locations]
        if not valid_goals:
            self.get_logger().error("No valid destinations found.")
            response.success = False
            response.message = "Error: No valid destinations found."
            return response

        # Set up delivery path: kitchen → valid goals → home
        self.pending_orders = ["kitchen"] + valid_goals + ["home"]
        self.get_logger().info(f"New Order Received: {valid_goals}")
        self.process_next_order()

        response.success = True
        response.message = "Successfully processed the goal list."
        return response

    def send_goal(self, location):
        """ Send navigation goal to action server """
        pose = self.get_pose_for_goal(location)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ Handle goal response from action server """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info("Goal Rejected!")
                self.goal_status = 0
                return

            self.get_logger().info("Goal Accepted!")
            self.goal_status = 1
            goal_handle.get_result_async().add_done_callback(self.result_callback)
        except Exception as e:
            self.get_logger().error(f"Error in goal_response_callback: {e}")

    def feedback_callback(self, feedback_msg):
        """ Process feedback from action server """
        if feedback_msg.feedback:
            self.goal_status = 2

    def result_callback(self, future):
        """ Handle goal completion """
        try:
            result = future.result()
            if result:
                self.goal_status = result.status
                if result.status == 4:
                    self.get_logger().info(f"{self.current_goal} reached successfully!")
                elif result.status == 5:
                    self.get_logger().warning("Goal Canceled!")
                elif result.status == 6:
                    self.get_logger().warning("Goal Aborted!")
                else:
                    self.get_logger().error(f"Goal execution failed! Status: {result.status}")
            else:
                self.get_logger().warning("Received an empty result!")
        except Exception as e:
            self.get_logger().error(f"Error in result_callback: {e}")

    def get_pose_for_goal(self, location):
        """ Get the PoseStamped message for a given location """
        x, y, yaw = self.locations[location]
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose


def main(args=None):
    """ Main function """
    rclpy.init(args=args)
    node = SimpleDeliveryRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
