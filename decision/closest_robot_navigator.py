import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient


class ClosestRobotNavigator(Node):
    def __init__(self, n):
        super().__init__('closest_robot_navigator')

        # Number of robots
        self.n = n

        # Action clients for computing paths and navigating
        self.compute_clients = {}
        self.navigate_clients = {}
        self.distances = {}

        for i in range(1, n + 1):
            robot_name = f'robot{i}'
            self.compute_clients[robot_name] = ActionClient(self, ComputePathToPose, f'/{robot_name}/compute_path_to_pose')
            self.navigate_clients[robot_name] = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
            self.distances[robot_name] = None  # Initialize distances

        # Target goal pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 1.0
        self.target_pose.pose.position.y = 1.0
        self.target_pose.pose.orientation.w = 1.0  # Face forward

        # Start computing paths
        self.get_logger().info(f"Sending compute path goals to {self.n} robots...")
        for robot_name in self.compute_clients.keys():
            self.send_compute_goal(robot_name)

    def send_compute_goal(self, robot_name):
        client = self.compute_clients[robot_name]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{robot_name} ComputePathToPose server not available.')
            self.distances[robot_name] = float('inf')
            self.check_and_move_closest_robot()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.target_pose
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda future: self.handle_compute_response(robot_name, future))

    def handle_compute_response(self, robot_name, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{robot_name} goal was rejected.")
            self.distances[robot_name] = float('inf')
            self.check_and_move_closest_robot()
            return

        goal_handle.get_result_async().add_done_callback(lambda future: self.process_compute_result(robot_name, future))

    def process_compute_result(self, robot_name, future):
        result = future.result().result
        if result:
            self.distances[robot_name] = self.compute_distance(result.path)
            self.get_logger().info(f"{robot_name} estimated path distance: {self.distances[robot_name]}")
        else:
            self.get_logger().error(f"Failed to compute path for {robot_name}.")
            self.distances[robot_name] = float('inf')

        self.check_and_move_closest_robot()

    def compute_distance(self, path: Path):
        # Calculate the total path distance
        distance = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            distance += ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) ** 0.5
        return distance

    def check_and_move_closest_robot(self):
        # Check if all distances are calculated
        if None in self.distances.values():
            return

        # Find the robot with the minimum distance
        closest_robot = min(self.distances, key=self.distances.get)
        if self.distances[closest_robot] == float('inf'):
            self.get_logger().error("No valid paths available for any robot.")
            return

        self.get_logger().info(f"{closest_robot} is closer. Sending navigation goal to {closest_robot}.")
        self.send_navigation_goal(closest_robot)

    def send_navigation_goal(self, robot_name):
        client = self.navigate_clients[robot_name]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{robot_name} NavigateToPose server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.target_pose
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_navigation_result)

    def handle_navigation_result(self, future):
        try:
            result = future.result()
            # Interpret status codes
            status = result.status
            if status == 4:  # SUCCEEDED
                self.get_logger().info("Robot successfully reached the goal!")
            elif status == 2:  # CANCELED
                self.get_logger().warn("Navigation was canceled. Check RViz for the robot's movement.")
            elif status == 5:  # ABORTED
                self.get_logger().error("Navigation aborted. There might be an issue with the goal or path.")
            else:
                self.get_logger().error(f"Navigation failed with unexpected status: {status}")
        except Exception as e:
            self.get_logger().error(f"Error while handling navigation result: {e}")
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ClosestRobotNavigator(n=2)  # Specify the number of robots here
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Graceful shutdown on Ctrl+C
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
