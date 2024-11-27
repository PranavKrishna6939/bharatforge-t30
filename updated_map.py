import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
import numpy as np

class OdomAndMapPublisher(Node):
    def __init__(self):
        super().__init__('odom_and_map_publisher')

        # Subscriptions to odometry and map topics
        self.sub_robot1_odom = self.create_subscription(
            Odometry, 
            '/robot1/odom', 
            self.robot1_odom_callback, 
            10
        )
        self.sub_robot2_odom = self.create_subscription(
            Odometry, 
            '/robot2/odom', 
            self.robot2_odom_callback, 
            10
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            10
        )

        # Publishers for synchronized data
        self.pub_robot1_synced = self.create_publisher(String, '/robot1/synced_data', 10)
        self.pub_robot2_synced = self.create_publisher(String, '/robot2/synced_data', 10)
        self.pub_updated_map = self.create_publisher(OccupancyGrid, '/updated_map', 10)

        # Timer to publish synchronized data at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_synced_data)

        # Variables to store the latest received data
        self.robot1_odom = None
        self.robot2_odom = None
        self.map_data = None

    def robot1_odom_callback(self, msg):
        """Callback to store the latest odometry data from Robot 1."""
        self.robot1_odom = msg

    def robot2_odom_callback(self, msg):
        """Callback to store the latest odometry data from Robot 2."""
        self.robot2_odom = msg

    def map_callback(self, msg):
        """Callback to store the latest map data."""
        self.map_data = msg

    def update_map_with_robots(self, map_matrix, robot1_pos, robot2_pos, resolution, origin):
        """Update the map grid with robot positions."""
        # Calculate the indices of the robots in the map grid
        robot1_x_idx = int((robot1_pos.x - origin.x) / resolution)
        robot1_y_idx = int((robot1_pos.y - origin.y) / resolution)

        robot2_x_idx = int((robot2_pos.x - origin.x) / resolution)
        robot2_y_idx = int((robot2_pos.y - origin.y) / resolution)

        # Ensure indices are within the bounds of the map
        map_height, map_width = map_matrix.shape
        if 0 <= robot1_x_idx < map_width and 0 <= robot1_y_idx < map_height:
            map_matrix[robot1_y_idx, robot1_x_idx] = 95  # Mark Robot 1 position
        if 0 <= robot2_x_idx < map_width and 0 <= robot2_y_idx < map_height:
            map_matrix[robot2_y_idx, robot2_x_idx] = 95 # Mark Robot 2 position

        return map_matrix

    def publish_synced_data(self):
        """Publish synchronized odometry and map data for both robots."""
        if self.robot1_odom and self.robot2_odom and self.map_data:
            # Process and format Robot 1 odometry data
            robot1_pos = self.robot1_odom.pose.pose.position
            robot1_info = f'[Robot1] Position: (x={robot1_pos.x}, y={robot1_pos.y}, z={robot1_pos.z})'

            # Process and format Robot 2 odometry data
            robot2_pos = self.robot2_odom.pose.pose.position
            robot2_info = f'[Robot2] Position: (x={robot2_pos.x}, y={robot2_pos.y}, z={robot2_pos.z})'

            # Extract and process map metadata and data
            map_width = self.map_data.info.width
            map_height = self.map_data.info.height
            map_resolution = self.map_data.info.resolution
            map_origin = self.map_data.info.origin.position  # Origin of the map (x, y, z)

            # Convert the map data to a numpy array (map data is 1D)
            map_matrix = np.array(self.map_data.data).reshape((map_height, map_width))

            # Update the map with robot positions
            updated_map = self.update_map_with_robots(map_matrix, robot1_pos, robot2_pos, map_resolution, map_origin)

            # Create a new OccupancyGrid message with the updated map data
            updated_map_data = updated_map.flatten().tolist()

            updated_map_msg = OccupancyGrid()
            updated_map_msg.header = self.map_data.header  # Retain original header information
            updated_map_msg.info = self.map_data.info  # Retain original map info
            updated_map_msg.data = updated_map_data  # Update map data with robot positions

            # Publish the updated map (even if it didn't change)
            self.pub_updated_map.publish(updated_map_msg)
            self.get_logger().info('Published updated map')

            # Combine all data into strings for robots
            combined_data_robot1 = f'{robot1_info}; Map published'
            combined_data_robot2 = f'{robot2_info}; Map published'

            # Publish the synchronized data
            # self.pub_robot1_synced.publish(String(data=combined_data_robot1))
            # self.pub_robot2_synced.publish(String(data=combined_data_robot2))
        else:
            # Log a warning if some data is missing
            self.get_logger().warning('Missing data: Ensure all topics are being published.')

def main(args=None):
    """Main function to initialize and run the ROS2 node."""
    rclpy.init(args=args)
    node = OdomAndMapPublisher()
    try:
        rclpy.spin(node)  # Keep the node running to listen for messages
    except KeyboardInterrupt:
        pass  # Handle graceful shutdown on Ctrl+C
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
