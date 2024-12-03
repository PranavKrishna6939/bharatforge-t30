import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re
import google.generativeai as genai

class ObjectCoordinateProcessor(Node):
    def __init__(self):
        super().__init__('object_coordinate_processor')
        
        # Initialize the LLM model with a hardcoded API key
        api_key = "AIzaSyAGAzglDEgQzr6X8FQP1Hu9CZkwhW-A_Zs"  # Replace this with your actual API key
        genai.configure(api_key=api_key)
        self.model = genai.GenerativeModel("gemini-1.5-flash")
        
        # Ask the user for a query only once
        self.user_query = None
        self.get_logger().info("Enter user prompt:")
        self.user_query = input()
        self.get_logger().info(f"User query updated: {self.user_query}")

        # Get the number of robots dynamically (default to 8 robots)
        self.num_robots = 8
        self.robot_odom = [None] * self.num_robots
        
        # Subscribe to odometry data for each robot dynamically
        self.sub_odom = [
            self.create_subscription(Odometry, f'/robot{i+1}/odom', self.odom_callback(i), 10)
            for i in range(self.num_robots)
        ]
        
        # Subscribe to the topic providing the object dictionary
        self.object_subscription = self.create_subscription(
            String,
            'object_history',
            self.listener_callback,
            10
        )

    def odom_callback(self, idx):
        """Generate a callback for a specific robot odometry."""
        def callback(msg):
            self.robot_odom[idx] = msg
        return callback

    def listener_callback(self, msg):
        """Callback to process object data and generate response."""
        if self.user_query is None:
            self.get_logger().warn("User query not set yet. Waiting for input.")
            return

        # Extract robot positions if available
        robot_positions = [
            odom.pose.pose.position if odom else None for odom in self.robot_odom
        ]
        
        # Parse incoming object data (using a mock object data dictionary)
        try:
            # The object_history is a string representation of a dictionary, so we use eval here.
            # Use eval carefully, as it can execute arbitrary code. Ensure that input is trusted.
            object_data = eval(msg.data)  # Evaluate the string into a Python dictionary
        except (SyntaxError, NameError) as e:
            self.get_logger().error(f"Failed to parse object data: {e}")
            return
        
        # Process with LLM
        response_text = self.query_llm(object_data, self.user_query, robot_positions)
        
        if response_text:
            # Extract coordinates from LLM response using regex
            pattern = r"\(\s*(\d+)\s*,\s*(\d+)\s*\)"  # Handle spaces
            match = re.search(pattern, response_text)
            if match:
                x, y = int(match.group(1)), int(match.group(2))
                self.get_logger().info(f"Extracted coordinate: x={x}, y={y}")
            else:
                self.get_logger().info("No coordinate found in the LLM response.")
        else:
            self.get_logger().warn("No response from LLM.")

    def query_llm(self, object_data, query, robot_positions):
        """Queries the LLM with the object data and user query."""
        prompt = (
            "Here is a dictionary of objects and their coordinates:\n"
            f"{object_data}\n\n"
            "and the current positions of bots from 1 to 8 are:\n"
            f"{robot_positions}\n\n"
            "Based on the user's query, identify the coordinates of the object in the form of (x, y). "
            f"Query: {query}"
        )
        try:
            response = self.model.generate_content(prompt)
            return response.text
        except Exception as e:
            self.get_logger().error(f"Error querying LLM: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    processor = ObjectCoordinateProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
