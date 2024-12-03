import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from std_msgs.msg import String


class ClosestRobotSelector(Node):
    def __init__(self):
        super().__init__('closest_robot_selector')

        # Action clients for computing paths and distances
        self.compute_clients = {}
        self.distances = {}

        # Subscribe to the robot status to get free robots
        self.free_robot_subscriber = self.create_subscription(String, 'robot_status', self.robot_status_callback, 10)

        # Keep track of free robots
        self.free_robots = {}

        # Subscriber for the goal pose
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

        # Publisher for the closest robot
        self.closest_robot_publisher = self.create_publisher(String, 'closest_robot', 10)

        # Variable to track the current goal
        self.current_goal = None
        self.published_closest_robot = False  # Flag to check if the closest robot has been published

    def robot_status_callback(self, msg):
        """Callback function to receive the status of free robots."""
        # Parse the free robots from the message and filter out "no robot is free"
        self.free_robots = {}
        for robot_status in msg.data.split(", "):
            robot_name, status = robot_status.split(": ")
            self.free_robots[robot_name] = True if status == "True" else False
        
        self.get_logger().info(f"Free robots: {self.free_robots}")
        self.check_and_publish_closest_robot()

    def goal_callback(self, msg):
        """Callback function for receiving the goal pose."""
        self.current_goal = msg
        self.get_logger().info(f"Received goal: x={msg.pose.position.x}, y={msg.pose.position.y}")

        if not self.free_robots:
            self.get_logger().info("No free robots available.")
            return

        # Reset the flag to allow publishing the closest robot for the new goal
        self.published_closest_robot = False

        # Send the compute path goal to free robots
        for robot_name, is_free in self.free_robots.items():
            if is_free:
                self.send_compute_goal(robot_name)

    def send_compute_goal(self, robot_name):
        """Send the goal to each free robot to compute the path."""
        if robot_name not in self.compute_clients:
            self.compute_clients[robot_name] = ActionClient(self, ComputePathToPose, f'/{robot_name}/compute_path_to_pose')

        client = self.compute_clients[robot_name]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{robot_name} ComputePathToPose server not available.')
            self.distances[robot_name] = float('inf')
            self.check_and_publish_closest_robot()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.current_goal
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda future: self.handle_compute_response(robot_name, future))

    def handle_compute_response(self, robot_name, future):
        """Handle the response from the compute path action."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{robot_name} goal was rejected.")
            self.distances[robot_name] = float('inf')
            self.check_and_publish_closest_robot()
            return

        goal_handle.get_result_async().add_done_callback(lambda future: self.process_compute_result(robot_name, future))

    def process_compute_result(self, robot_name, future):
        """Process the result from the path computation."""
        result = future.result().result
        if result:
            self.distances[robot_name] = self.compute_distance(result.path)
            self.get_logger().info(f"{robot_name} estimated path distance: {self.distances[robot_name]}")
        else:
            self.get_logger().error(f"Failed to compute path for {robot_name}.")
            self.distances[robot_name] = float('inf')

        self.check_and_publish_closest_robot()

    def compute_distance(self, path):
        """Compute the total distance of the path."""
        distance = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            distance += ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) ** 0.5
        return distance

    def check_and_publish_closest_robot(self):
        """Check and publish the closest free robot, only after all robots have completed their distance calculation."""
        if not self.free_robots:
            self.get_logger().info("No free robots available.")
            return

        # Filter distances for only the free robots
        free_robot_distances = {
            robot_name: self.distances[robot_name] for robot_name, is_free in self.free_robots.items() if is_free and self.distances.get(robot_name) is not None
        }

        if not free_robot_distances:
            self.get_logger().info("No valid paths for free robots.")
            return

        # Check if all free robots have completed their distance calculation
        if len(free_robot_distances) < len([robot_name for robot_name, is_free in self.free_robots.items() if is_free]):
            # Not all free robots have completed, return early
            return

        # Find the closest free robot based on the shortest distance
        closest_robot = min(free_robot_distances, key=free_robot_distances.get)
        if free_robot_distances[closest_robot] == float('inf'):
            self.get_logger().info("No valid path for the closest robot.")
            return

        # Publish the closest robot only if it has not been published yet
        if not self.published_closest_robot:
            closest_distance = free_robot_distances[closest_robot]
            self.get_logger().info(f"Closest free robot: {closest_robot}, Distance: {closest_distance:.2f}")
            
            # Publish the closest robot and the goal coordinates
            closest_robot_info = f"{closest_robot}, Goal: x={self.current_goal.pose.position.x}, y={self.current_goal.pose.position.y}"
            self.closest_robot_publisher.publish(String(data=closest_robot_info))
            self.published_closest_robot = True


def main(args=None):
    rclpy.init(args=args)

    # Start the node
    node = ClosestRobotSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

