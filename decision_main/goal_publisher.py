import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import threading


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Publisher for the goal_pose topic using PoseArray
        self.publisher = self.create_publisher(PoseArray, 'goal_pose', 10)

        # Dictionary of predefined locations with multiple coordinates
        self.locations = {
            "fire extinguisher": [
                {"x": 1.0, "y": -8.0},
                {"x": 1.2, "y": 2.3}
            ],
            "coffee machine": [
                {"x": 1.2, "y": 2.3}
            ],
            "water cooler": [
                {"x": 1.0, "y": -8.0},
                {"x": 1.2, "y": 2.3},
                {"x": 5.6, "y": 1.5}
            ]
        }

        # Start the input loop in a separate thread
        input_thread = threading.Thread(target=self.input_loop, daemon=True)
        input_thread.start()

    def input_loop(self):
        while rclpy.ok():
            try:
                # Get user input synchronously
                item = input("\nEnter the name of the location (or 'exit' to quit): ").strip().lower()

                if item == "exit":
                    self.get_logger().info("Exiting goal publisher...")
                    rclpy.shutdown()
                    return

                if item in self.locations:
                    self.publish_goals(item)
                else:
                    self.get_logger().error(f"Invalid location: {item}. Please try again.")
            except EOFError:
                # Handle end-of-file (e.g., Ctrl+D)
                self.get_logger().info("EOF received. Exiting goal publisher...")
                rclpy.shutdown()
                return
            except Exception as e:
                self.get_logger().error(f"An error occurred: {e}")
                rclpy.shutdown()
                return

    def publish_goals(self, item_name):
        coordinates = self.locations[item_name]
        self.get_logger().info(f"Publishing goals for {item_name}:")

        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'  # Ensure the frame matches your setup
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for idx, coord in enumerate(coordinates, start=1):
            pose = Pose()
            pose.position.x = float(coord["x"])
            pose.position.y = float(coord["y"])
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)
            self.get_logger().info(f"  {idx}. x={coord['x']}, y={coord['y']}")

        self.publisher.publish(pose_array)
        self.get_logger().info(f"Published PoseArray with {len(coordinates)} poses.")

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("GoalPublisher node destroyed.")


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
