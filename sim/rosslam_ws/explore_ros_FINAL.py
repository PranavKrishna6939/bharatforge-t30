import numpy as np
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import subprocess
import time

# Environment parameters
MAP_SIZE = 900
ROBOT_PIXEL_START = 99  # Robot1: 99, Robot2: 98, etc.
WAIT_TIME = 5  # Seconds to wait after reaching goals


# Define the neural network model (Actor-Critic)
class ActorCritic(nn.Module):
    def __init__(self, input_size, num_bots=1):
        super(ActorCritic, self).__init__()
        self.num_bots = num_bots
        self.fc = nn.Linear(input_size, 256)
        self.policy_head = nn.Linear(256, num_bots * 2)  # x and y for each bot
        self.value_head = nn.Linear(256, 1)

    def forward(self, x, bot_positions):
        x = torch.relu(self.fc(x))
        policy = self.policy_head(x)
        value = self.value_head(x)
        return policy, value


def quaternion_from_euler(roll, pitch, yaw):
    import math
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
         math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
         math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


class ExplorationEnv(Node):
    def __init__(self, number_of_bots):
        super().__init__('exploration_env')
        self.number_of_bots = number_of_bots
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/updated_map',
            self.map_callback,
            10
        )
        self.map_data = None
        self.map_info = None
        self.bot_positions = [None for _ in range(self.number_of_bots)]
        input_size = MAP_SIZE * MAP_SIZE  # Flattened map
        self.model = ActorCritic(input_size, self.number_of_bots)
        self.model.eval()
        self.get_logger().info('ExplorationEnv node has been started.')

    def map_callback(self, msg):
        try:
            msg.info.height = MAP_SIZE
            msg.info.width = MAP_SIZE
            expected_size = msg.info.height * msg.info.width
            actual_size = len(msg.data)

            if expected_size != actual_size:
                self.get_logger().error(
                    f"Map size mismatch: Expected {expected_size}, got {actual_size}."
                )
                return

            # Reshape the data
            self.map_data = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            )
            self.map_info = msg.info
            self.process_map()  # Run the method for map processing
        except ValueError as e:
            self.get_logger().error(f"Error reshaping map data: {str(e)}")

    def process_map(self):
        if self.map_data is None:
            self.get_logger().info("Map data is None.")
            return
        if self.map_info is None:
            self.get_logger().warn('Map info is not available yet.')
            return

        # Find robot positions
        for i in range(self.number_of_bots):
            bot_pixel_value = ROBOT_PIXEL_START - i
            positions = np.argwhere(self.map_data == bot_pixel_value)
            if positions.size > 0:
                self.bot_positions[i] = positions[0]  # (y, x)
                self.get_logger().info(f"Robot {i+1} found at position: {self.bot_positions[i]}")
            else:
                self.bot_positions[i] = [0, 0]
                self.get_logger().warn(f'Robot {i+1} position not found in the map.')

        # Prepare input for the model
        flattened_map = self.map_data.flatten().astype(np.float32)
        input_tensor = torch.tensor(flattened_map).unsqueeze(0)  # Shape: [1, input_size]
        bot_positions_flat = np.array(self.bot_positions).flatten().astype(np.float32)
        bot_positions_tensor = torch.tensor(bot_positions_flat).unsqueeze(0)  # Shape: [1, num_bots*2]

        # Get next positions from the model
        with torch.no_grad():
            actions, _ = self.model(input_tensor, bot_positions_tensor)
        actions_np = actions.squeeze(0).numpy().reshape(self.number_of_bots, 2)

        for i in range(self.number_of_bots):
            print(f"Robot {i+1} predicted pixel position: x={actions_np[i][0]}, y={actions_np[i][1]}")

        # Convert pixel positions to map coordinates and send navigation goals
        for i in range(self.number_of_bots):
            x_pixel, y_pixel = actions_np[i]
            robot_namespace = f'/robot{i+1}'
            x, y, theta = x_pixel, y_pixel, 0

            # Send the navigation goal
            self.send_goal(robot_namespace, x, y, theta)

        # Wait additional seconds for environment to localize and map
        self.get_logger().info(f'Waiting for {WAIT_TIME} seconds...')
        time.sleep(WAIT_TIME)

    def send_goal(self, robot_namespace, x, y, theta):
        # Convert theta from degrees to radians
        theta_rad = theta * (np.pi / 180.0)
        q = quaternion_from_euler(0, 0, theta_rad)

        # Action server name with namespace
        action_server_name = f'{robot_namespace}/navigate_to_pose'

        self.get_logger().info(f'Using action server: {action_server_name}')

        # Construct the command
        command = [
            'ros2', 'action', 'send_goal', action_server_name, 'nav2_msgs/action/NavigateToPose',
            f'{{pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}}}}}}}}}'
        ]

        self.get_logger().info(f'Sending goal to navigate to the specified pose: x={x}, y={y}, theta={theta} degrees')

        # Execute the command synchronously
        result = subprocess.run(command, capture_output=True, text=True)

        if result.returncode == 0:
            self.get_logger().info('Goal sent successfully.')
            self.get_logger().info(result.stdout)
        else:
            self.get_logger().error('Failed to send goal.')
            self.get_logger().error(result.stderr)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)  # Ensure ROS 2 is initialized first
    number_of_bots = 2
    exploration_env = ExplorationEnv(number_of_bots)
    try:
        rclpy.spin(exploration_env)
    except KeyboardInterrupt:
        pass
    exploration_env.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
