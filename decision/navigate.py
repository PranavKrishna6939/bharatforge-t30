import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient


class ClosestRobotNavigator(Node):
    def __init__(self):
        super().__init__('closest_robot_navigator')

        # Action clients for computing paths and navigating
        self.robot1_compute_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.robot2_compute_client = ActionClient(self, ComputePathToPose, '/robot2/compute_path_to_pose')
        self.robot1_navigate_client = ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose')
        self.robot2_navigate_client = ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')

        # Define the target goal
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 1.0  # Set your desired goal's X coordinate
        self.target_pose.pose.position.y = 1.0  # Set your desired goal's Y coordinate
        self.target_pose.pose.orientation.w = 1.0

        # Variables to store computed distances
        self.robot1_distance = None
        self.robot2_distance = None

        # Start the distance calculation
        self.get_logger().info("Calculating path distances for both robots...")
        self.send_compute_goal_to_robot1()
        self.send_compute_goal_to_robot2()

    def send_compute_goal_to_robot1(self):
        if not self.robot1_compute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Robot1 ComputePathToPose server not available.')
            self.robot1_distance = float('inf')  # Assign a very high value
            self.check_and_move_closest_robot()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.target_pose
        future = self.robot1_compute_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_robot1_response)

    def send_compute_goal_to_robot2(self):
        if not self.robot2_compute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Robot2 ComputePathToPose server not available.')
            self.robot2_distance = float('inf')  # Assign a very high value
            self.check_and_move_closest_robot()
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.target_pose
        future = self.robot2_compute_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_robot2_response)

    def handle_robot1_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Robot1 goal was not accepted.")
            self.robot1_distance = float('inf')
            self.check_and_move_closest_robot()
            return

        self.get_logger().info("Robot1 goal accepted. Calculating distance...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.process_robot1_result)

    def handle_robot2_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Robot2 goal was not accepted.")
            self.robot2_distance = float('inf')
            self.check_and_move_closest_robot()
            return

        self.get_logger().info("Robot2 goal accepted. Calculating distance...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.process_robot2_result)

    def process_robot1_result(self, future):
        result = future.result().result
        self.robot1_distance = self.compute_distance(result.path)
        self.get_logger().info(f'Robot1 path distance: {self.robot1_distance}')
        self.check_and_move_closest_robot()

    def process_robot2_result(self, future):
        result = future.result().result
        self.robot2_distance = self.compute_distance(result.path)
        self.get_logger().info(f'Robot2 path distance: {self.robot2_distance}')
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
        # Check if both distances are calculated
        if self.robot1_distance is None or self.robot2_distance is None:
            return

        # Determine the closest robot and send the navigation goal
        if self.robot1_distance < self.robot2_distance:
            self.get_logger().info("Robot1 is closer. Sending navigation goal to Robot1.")
            self.send_navigation_goal(self.robot1_navigate_client)
        elif self.robot2_distance < self.robot1_distance:
            self.get_logger().info("Robot2 is closer. Sending navigation goal to Robot2.")
            self.send_navigation_goal(self.robot2_navigate_client)
        else:
            self.get_logger().info("Both robots have equal distance. Defaulting to Robot1.")
            self.send_navigation_goal(self.robot1_navigate_client)

    def send_navigation_goal(self, navigate_client):
        if not navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.target_pose

        future = navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self.handle_navigation_result)

    def handle_navigation_result(self, future):
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("Robot successfully reached the goal!")
        else:
            self.get_logger().error(f"Navigation failed with status: {result.status}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ClosestRobotNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()

