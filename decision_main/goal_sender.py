import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')

        # Subscriber for closest robot and goal (from ClosestRobotSelector)
        self.closest_robot_subscriber = self.create_subscription(
            String, 'closest_robot', self.closest_robot_callback, 10
        )

        # Publisher to announce when the robot reaches the goal
        self.robot_reached_goal_publisher = self.create_publisher(String, 'robot_reached_goal', 10)

        # Variables to store goal, closest robot, and action clients for each robot
        self.current_goal = None
        self.closest_robot = None
        self.navigate_clients = {}

    def closest_robot_callback(self, msg):
        """Callback function to receive the closest robot and its goal coordinates."""
        # Parse the message to get the robot name and goal coordinates
        data = msg.data.split(", Goal: ")
        robot_name = data[0].strip()  # Extract robot name
        goal_coordinates = data[1] if len(data) > 1 else ""

        # Split the goal coordinates into x and y values
        try:
            goal_data = goal_coordinates.split(", ")
            goal_x = float(goal_data[0].split("=")[1])
            goal_y = float(goal_data[1].split("=")[1])

            self.get_logger().info(f"Closest robot: {robot_name}, Goal: x={goal_x}, y={goal_y}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse the goal coordinates: {e}")
            return

        # Create a PoseStamped message for the goal
        self.current_goal = PoseStamped()
        self.current_goal.header.stamp = self.get_clock().now().to_msg()
        self.current_goal.header.frame_id = 'map'
        self.current_goal.pose.position.x = goal_x
        self.current_goal.pose.position.y = goal_y
        self.current_goal.pose.orientation.w = 1.0  # No rotation (quaternion)

        # Store the closest robot
        self.closest_robot = robot_name

        # Dynamically create an action client for the closest robot
        self.create_action_client(self.closest_robot)

        # Send the closest robot to the goal
        self.send_robot_to_goal(self.closest_robot)

    def create_action_client(self, robot_name):
        """Dynamically create an action client for the given robot name."""
        # Check if the action client for the robot already exists
        if robot_name not in self.navigate_clients:
            self.navigate_clients[robot_name] = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
            self.get_logger().info(f"Created action client for {robot_name}")

    def send_robot_to_goal(self, robot_name):
        """Send the closest robot to the goal using the NavigateToPose action."""
        client = self.navigate_clients.get(robot_name)

        if client is None:
            self.get_logger().error(f"No action client found for {robot_name}")
            return

        # Wait for the server to be available
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{robot_name} NavigateToPose server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda future: self.handle_navigation_response(robot_name, future))

    def handle_navigation_response(self, robot_name, future):
        """Handle the response from the NavigateToPose action."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{robot_name} goal was rejected.")
            return

        goal_handle.get_result_async().add_done_callback(lambda future: self.process_navigation_result(robot_name, future))

    def process_navigation_result(self, robot_name, future):
        """Process the result from the NavigateToPose action."""
        result = future.result().result
        if result:
            self.get_logger().info(f"{robot_name} successfully reached the goal!")
            self.publish_robot_reached_goal(robot_name)
        else:
            self.get_logger().error(f"{robot_name} failed to reach the goal.")

    def publish_robot_reached_goal(self, robot_name):
        """Publish the robot's name when it reaches the goal."""
        msg = String()
        msg.data = robot_name
        self.robot_reached_goal_publisher.publish(msg)
        self.get_logger().info(f"Published: {robot_name} has reached the goal.")


def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

