import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

class PathDistanceCalculator(Node):
    def __init__(self):
        super().__init__('path_distance_calculator')
        self.robot1_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.robot2_client = ActionClient(self, ComputePathToPose, '/robot2/compute_path_to_pose')
        self.target_pose = PoseStamped()

        # Set target pose (Example: x=1.0, y=1.0, map frame)
        self.target_pose.header.frame_id = 'map'
        self.target_pose.pose.position.x = 1.0
        self.target_pose.pose.position.y = 1.0
        self.target_pose.pose.orientation.w = 1.0  # Face forward

    def send_goal(self):
        # Wait for both action servers to be available
        self.robot1_client.wait_for_server()
        self.robot2_client.wait_for_server()

        # Send goals to both robots
        self.get_logger().info('Sending goals to both robots...')
        robot1_future = self.robot1_client.send_goal_async(
            ComputePathToPose.Goal(goal=self.target_pose)
        )
        robot2_future = self.robot2_client.send_goal_async(
            ComputePathToPose.Goal(goal=self.target_pose)
        )

        # Process responses asynchronously
        robot1_future.add_done_callback(self.handle_robot1_response)
        robot2_future.add_done_callback(self.handle_robot2_response)

        self.robot1_distance = None
        self.robot2_distance = None

    def handle_robot1_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Robot 1 goal was not accepted")
            self.robot1_distance = float('inf')  # Assign a very high value
            self.check_results()
            return

        self.get_logger().info("Robot 1 goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.process_robot1_result)

    def process_robot1_result(self, future):
        result = future.result().result
        self.robot1_distance = self.compute_distance(result.path)
        self.get_logger().info(f'Robot 1 estimated distance: {self.robot1_distance} meters')
        self.check_results()

    def handle_robot2_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Robot 2 goal was not accepted")
            self.robot2_distance = float('inf')  # Assign a very high value
            self.check_results()
            return

        self.get_logger().info("Robot 2 goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.process_robot2_result)

    def process_robot2_result(self, future):
        result = future.result().result
        self.robot2_distance = self.compute_distance(result.path)
        self.get_logger().info(f'Robot 2 estimated distance: {self.robot2_distance} meters')
        self.check_results()

    def compute_distance(self, path: Path):
        # Calculate the total path distance
        distance = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            distance += ((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2) ** 0.5
        return distance

    def check_results(self):
        # Check if both distances are calculated
        if self.robot1_distance is not None and self.robot2_distance is not None:
            if self.robot1_distance < self.robot2_distance:
                self.get_logger().info('Robot 1 requires the minimum distance.')
            elif self.robot2_distance < self.robot1_distance:
                self.get_logger().info('Robot 2 requires the minimum distance.')
            else:
                self.get_logger().info('Both robots require the same distance.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PathDistanceCalculator()
    node.send_goal()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
