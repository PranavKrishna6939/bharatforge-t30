import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Publisher for the goal_pose topic
        self.publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Static dictionary of predefined coordinates
        self.locations = {
            "fire extinguisher": {"x": 1.0, "y": -8.0},
            "coffee machine": {"x": 2.0, "y": 1.5},
            "water cooler": {"x": 1.2, "y": 2.3}
        }

        # Start the publishing loop
        self.timer = self.create_timer(1.0, self.publish_goal_callback)  # 1 Hz timer

    def publish_goal_callback(self):
        # Ask for input continuously
        item = input("\nEnter the name of the location (or 'exit' to quit): ").strip().lower()

        # Check for exit condition
        if item == "exit":
            self.get_logger().info("Exiting goal publisher...")
            rclpy.shutdown()
            return

        if item in self.locations:
            self.publish_goal(item)
        else:
            self.get_logger().error(f"Invalid location: {item}. Please try again.")

    def publish_goal(self, item_name):
        # Get coordinates from the dictionary
        coordinates = self.locations[item_name]

        # Create a PoseStamped message
        goal = PoseStamped()
        goal.header.frame_id = 'map'  # Ensure the frame matches your setup
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(coordinates["x"])
        goal.pose.position.y = float(coordinates["y"])
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # Publish the goal
        self.publisher.publish(goal)
        self.get_logger().info(f"Publishing goal for {item_name}: x={coordinates['x']}, y={coordinates['y']}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GoalPublisher Node Stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

