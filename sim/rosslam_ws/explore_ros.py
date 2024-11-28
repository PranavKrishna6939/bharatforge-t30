# explore_ros.py

import numpy as np
import torch
import torch.nn as nn
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import threading
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
        self.env = None
        self.current_position = None

    def forward(self, x, bot_positions):
        x = torch.relu(self.fc(x))
        policy = self.policy_head(x)
        value = self.value_head(x)
        return policy, value

class ExplorationEnv(Node):
    def __init__(self, number_of_bots):
        super().__init__('exploration_env')
        self.number_of_bots = number_of_bots
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/updated_map',
            self.map_callback,
            10)
        self.map_data = None
        self.bot_positions = [None for _ in range(self.number_of_bots)]
        input_size = MAP_SIZE * MAP_SIZE  # Flattened map
        self.model = ActorCritic(input_size, self.number_of_bots)
        # self.model.load_state_dict(torch.load('model_weights.pth', map_location=torch.device('cpu')))
        
        self.model.eval()
        self.action_clients = [
            ActionClient(self, NavigateToPose, f'/robot{i+1}/navigate_to_pose')
            for i in range(self.number_of_bots)
        ]
        self.get_logger().info('ExplorationEnv node has been started.')

    def map_callback(self, msg):
        # Update the occupancy map
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.process_map()

    def process_map(self):
        if self.map_data is None:
            return

        # Find robot positions
        for i in range(self.number_of_bots):
            bot_pixel_value = ROBOT_PIXEL_START - i
            positions = np.argwhere(self.map_data == bot_pixel_value)
            if positions.size > 0:
                self.bot_positions[i] = positions[0]  # (y, x)
            else:
                self.get_logger().warn(f'Robot {i+1} position not found in the map.')
                return

        # Prepare input for the model
        flattened_map = self.map_data.flatten().astype(np.float32)
        input_tensor = torch.tensor(flattened_map).unsqueeze(0)  # Shape: [1, input_size]
        bot_positions_flat = np.array(self.bot_positions).flatten().astype(np.float32)
        bot_positions_tensor = torch.tensor(bot_positions_flat).unsqueeze(0)  # Shape: [1, num_bots*2]

        # Get next positions from the model
        with torch.no_grad():
            actions, _ = self.model(input_tensor, bot_positions_tensor)
        actions_np = actions.squeeze(0).numpy().reshape(self.number_of_bots, 2)

        # Convert pixel positions to map coordinates and send navigation goals
        threads = []
        for i in range(self.number_of_bots):
            x_pixel, y_pixel = actions_np[i]
            x_pixel = int(np.clip(x_pixel, 0, MAP_SIZE - 1))
            y_pixel = int(np.clip(y_pixel, 0, MAP_SIZE - 1))
            map_x, map_y = self.pixel_to_map_coordinates(x_pixel, y_pixel, msg_info=self.get_map_info())

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = map_x
            pose.pose.position.y = map_y
            pose.pose.orientation.w = 1.0

            thread = threading.Thread(target=self.send_goal, args=(i, pose))
            threads.append(thread)
            thread.start()

        # Wait for all threads to finish
        for thread in threads:
            thread.join()

        # Wait additional 5 seconds for environment to localize and map
        self.get_logger().info(f'Waiting for {WAIT_TIME} seconds...')
        time.sleep(WAIT_TIME)

    def get_map_info(self):
        # Retrieve the latest map info
        # Assumes that map_callback has been called at least once
        return self.create_subscription(
            OccupancyGrid,
            '/updated_map',
            lambda msg: msg.info,
            10
        )

    def pixel_to_map_coordinates(self, x_pixel, y_pixel, msg_info=None):
        if msg_info is None:
            return 0.0, 0.0
        resolution = 0.05
        origin_x = msg_info.origin.position.x
        origin_y = msg_info.origin.position.y
        map_x = x_pixel * resolution + origin_x
        map_y = y_pixel * resolution + origin_y
        return map_x, map_y

    def send_goal(self, robot_index, pose):
        action_client = self.action_clients[robot_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if not action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'Action server for robot {robot_index+1} not available.')
            return

        future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for robot {robot_index+1}.')
            return

        self.get_logger().info(f'Goal accepted for robot {robot_index+1}.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 4:
            self.get_logger().info(f'Robot {robot_index+1} reached the goal.')
        else:
            self.get_logger().error(f'Robot {robot_index+1} failed to reach the goal.')

def main(args=None):
    rclpy.init(args=args)
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
