import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
import threading


class ClosestRobotSelector(Node):
    def __init__(self):
        super().__init__('closest_robot_selector')

        # Action clients for computing paths and distances
        self.compute_clients = {}
        self.distances = {}  # Key: (robot_name, coordinate_index), Value: distance

        # Subscribe to the robot status to get free robots
        self.free_robot_subscriber = self.create_subscription(
            String,
            'robot_status',
            self.robot_status_callback,
            10
        )

        # Keep track of free robots
        self.free_robots = {}
        self.free_robots_lock = threading.Lock()

        # Subscriber for the goal pose (PoseArray)
        self.goal_subscriber = self.create_subscription(
            PoseArray,
            'goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for the closest robot
        self.closest_robot_publisher = self.create_publisher(
            String,
            'closest_robot',
            10
        )

        # Variable to track the current goals
        self.current_goals = []
        self.current_goals_lock = threading.Lock()
        self.published_closest_robot = False  # Flag to check if the closest robot has been published

        # Lock to prevent race conditions when accessing distances
        self.distances_lock = threading.Lock()

    def robot_status_callback(self, msg):
        """Callback function to receive the status of free robots."""
        with self.free_robots_lock:
            # Parse the free robots from the message and filter out "no robot is free"
            self.free_robots = {}
            for robot_status in msg.data.split(", "):
                if not robot_status:
                    continue
                parts = robot_status.split(": ")
                if len(parts) != 2:
                    self.get_logger().error(f"Invalid robot status format: {robot_status}")
                    continue
                robot_name, status = parts
                self.free_robots[robot_name] = status.lower() == "true"

        self.get_logger().info(f"Free robots: {self.free_robots}")
        # Optionally, you can trigger re-evaluation if robot statuses change
        # For simplicity, we proceed when a new goal is received

    def goal_callback(self, msg):
        """Callback function for receiving the goal poses."""
        # Start a new thread to process the goals
        threading.Thread(target=self.process_goals, args=(msg,), daemon=True).start()

    def process_goals(self, msg):
        """Process the received PoseArray goals."""
        with self.current_goals_lock:
            self.current_goals = msg.poses
            self.get_logger().info(f"Received PoseArray with {len(self.current_goals)} poses.")

        with self.free_robots_lock:
            if not self.free_robots:
                self.get_logger().info("No free robots available.")
                return

        if not self.current_goals:
            self.get_logger().info("Received empty PoseArray.")
            return

        # Reset previous data
        with self.distances_lock:
            self.distances = {}
            self.published_closest_robot = False

        # Send ComputePathToPose goals for all free robots and all coordinates
        for coord_idx, pose in enumerate(self.current_goals):
            with self.free_robots_lock:
                free_robot_names = [robot for robot, is_free in self.free_robots.items() if is_free]

            for robot_name in free_robot_names:
                self.send_compute_goal(robot_name, coord_idx, pose)

    def send_compute_goal(self, robot_name, coord_idx, pose):
        """Send the goal to each free robot to compute the path."""
        if robot_name not in self.compute_clients:
            self.compute_clients[robot_name] = ActionClient(
                self,
                ComputePathToPose,
                f'/{robot_name}/compute_path_to_pose'
            )

        client = self.compute_clients[robot_name]

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{robot_name} ComputePathToPose server not available.')
            with self.distances_lock:
                self.distances[(robot_name, coord_idx)] = float('inf')
            self.check_and_publish_closest_robot()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.create_pose_stamped(pose)

        self.get_logger().info(f"Sending ComputePathToPose goal to {robot_name} for coordinate index {coord_idx}.")
        client.send_goal_async(
            goal_msg,
            feedback_callback=None
        ).add_done_callback(lambda future: self.handle_compute_response(robot_name, coord_idx, future))

    def handle_compute_response(self, robot_name, coord_idx, future):
        """Handle the response from the compute path action."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{robot_name} ComputePathToPose goal was rejected.")
            with self.distances_lock:
                self.distances[(robot_name, coord_idx)] = float('inf')
            self.check_and_publish_closest_robot()
            return

        self.get_logger().info(f"{robot_name} ComputePathToPose goal accepted.")
        goal_handle.get_result_async().add_done_callback(
            lambda fut: self.process_compute_result(robot_name, coord_idx, fut)
        )

    def process_compute_result(self, robot_name, coord_idx, future):
        """Process the result from the path computation."""
        try:
            result = future.result().result
            if result and result.path:
                distance = self.compute_distance(result.path)
                with self.distances_lock:
                    self.distances[(robot_name, coord_idx)] = distance
                self.get_logger().info(
                    f"{robot_name} estimated path distance to coordinate {coord_idx}: {distance:.2f}"
                )
            else:
                self.get_logger().error(f"Failed to compute path for {robot_name} to coordinate {coord_idx}.")
                with self.distances_lock:
                    self.distances[(robot_name, coord_idx)] = float('inf')
        except Exception as e:
            self.get_logger().error(f"Exception while processing result for {robot_name} to coordinate {coord_idx}: {e}")
            with self.distances_lock:
                self.distances[(robot_name, coord_idx)] = float('inf')

        self.check_and_publish_closest_robot()

    def compute_distance(self, path):
        """Compute the total distance of the path."""
        distance = 0.0
        poses = path.poses
        for i in range(len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            distance += ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) ** 0.5
        return distance

    def check_and_publish_closest_robot(self):
        """Check and publish the closest free robot and corresponding coordinate."""
        with self.distances_lock:
            if not self.current_goals:
                return  # No current goals to process

            # Calculate total expected distance computations
            with self.free_robots_lock:
                total_pairs = len(self.current_goals) * len([r for r in self.free_robots if self.free_robots[r]])

            completed_pairs = len(self.distances)

            if completed_pairs < total_pairs:
                # Not all computations are done yet
                return

            if not self.distances:
                self.get_logger().info("No valid paths for free robots.")
                return

            # Find the (robot, coordinate) pair with the minimum distance
            closest_pair = min(self.distances, key=self.distances.get)
            closest_distance = self.distances[closest_pair]

            if closest_distance == float('inf'):
                self.get_logger().info("No valid path found for any robot-coordinate pair.")
                return

            closest_robot, coord_idx = closest_pair
            closest_pose = self.current_goals[coord_idx]

            self.get_logger().info(
                f"Closest free robot: {closest_robot}, Coordinate Index: {coord_idx}, Distance: {closest_distance:.2f}"
            )

            # Publish the closest robot and corresponding coordinate
            closest_robot_info = (
                f"{closest_robot}, Goal: x={closest_pose.position.x}, y={closest_pose.position.y}"
            )
            self.closest_robot_publisher.publish(String(data=closest_robot_info))
            self.get_logger().info(f"Published closest robot info: {closest_robot_info}")

            # Optionally, mark the robot as busy
            # with self.free_robots_lock:
            #     self.free_robots[closest_robot] = False

    def create_pose_stamped(self, pose):
        """Helper function to create a PoseStamped message from a Pose."""
        from geometry_msgs.msg import PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("ClosestRobotSelector node destroyed.")


def main(args=None):
    rclpy.init(args=args)

    # Start the node
    node = ClosestRobotSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ClosestRobotSelector Node Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
