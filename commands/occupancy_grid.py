import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,  # The message type
            '/map',  # The topic to subscribe to
            self.listener_callback,  # The callback function
            10  # Queue size (number of messages to store)
        )

    def listener_callback(self, msg):
        # Get map information (metadata)
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        # Extract the map data and convert it to a 2D numpy array
        map_data = np.array(msg.data).reshape((height, width))

        # For debugging: print the map as a 2D matrix
        self.get_logger().info(f'Map (2D matrix) with resolution {resolution} m/pixel:')
        print(map_data)
        print(map_data.shape)

def main(args=None):
    rclpy.init(args=args)
    map_subscriber = MapSubscriber()
    rclpy.spin(map_subscriber)  # Keeps the node running and listening for messages
    rclpy.shutdown()

if __name__ == '__main__':
    main()

