import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,  # The message type
            '/robot1/odom',  # The topic to subscribe to
            self.listener_callback,  # The callback function
            10  # Queue size (number of messages to store)
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract and log the position and orientation data from the Odometry message
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.get_logger().info(f'Received Odometry: Position: (x={position.x}, y={position.y}, z={position.z})')

def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()
    rclpy.spin(odom_subscriber)  # Keeps the node running and listening for messages
    rclpy.shutdown()

if __name__ == '__main__':
    main()

