import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import torch
import torch.nn as nn
import torch.optim as optim
from scipy.spatial import ConvexHull
from matplotlib.path import Path
from scipy.ndimage import binary_erosion, binary_dilation
from collections import deque

# Environment parameters
MAP_SIZE = 900  # Map size of 900x900
MAX_AREA = (MAP_SIZE * MAP_SIZE) // 2  # Occupied area at least half of the map
MAX_TIMESTEPS = 5000
MAX_RANGE = 10
NUMBER_OF_BOTS = 3  # Number of robots

# Rewards
REWARD_NEW_PIXEL = 1.0  # Adjusted for percentage
PENALTY_UNEXPLORED = -0.5
PENALTY_COLLISION = -10
PENALTY_OUT_OF_BOUNDS = -100

# Define the environment
class ExplorationEnv:
    def __init__(self, number_of_bots=NUMBER_OF_BOTS):
        self.number_of_bots = number_of_bots
        self.generate_map()
        self.explored_map = np.full_like(self.actual_map, -1)  # -1: Unexplored, 0: Free, 100: Obstacle
        self.bot_positions = self.initialize_bot_positions()
        self.update_explored_map_with_bots()
        self.steps = 0

    def generate_map(self):
        grid_size = MAP_SIZE  # Map size of 900x900
        self.actual_map = np.ones((grid_size, grid_size), dtype=np.int32) * -1  # Initialize as invalid cells

        # Generate random polygon until it occupies at least half of the MAX_AREA
        while True:
            num_vertices = np.random.randint(4, 15)
            angles = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
            radius = np.random.uniform(grid_size * 0.2, grid_size * 0.4, size=num_vertices)
            center = (grid_size // 2, grid_size // 2)
            points = np.vstack((center[0] + radius * np.cos(angles),
                                center[1] + radius * np.sin(angles))).T

            # Create a path and fill the polygon
            xv, yv = np.meshgrid(np.arange(grid_size), np.arange(grid_size), indexing='ij')
            xv = xv.flatten()
            yv = yv.flatten()
            coords = np.vstack((xv, yv)).T
            path = Path(points)
            mask = path.contains_points(coords)
            mask = mask.reshape((grid_size, grid_size))
            self.actual_map[mask] = 0  # Free space

            # Check if the free space occupies at least half of MAX_AREA
            free_space = np.sum(self.actual_map == 0)
            if free_space >= MAX_AREA // 2:
                break
            else:
                # Reset the map and try again
                self.actual_map = np.ones((grid_size, grid_size), dtype=np.int32) * -1

        # Create a 2-pixel wide boundary around the polygon
        boundary = mask ^ binary_erosion(mask, structure=np.ones((3, 3)), iterations=1)
        boundary_2pixel = binary_dilation(boundary, structure=np.ones((3, 3)), iterations=1)
        self.actual_map[boundary_2pixel] = 100  # Obstacles

    def initialize_bot_positions(self):
        positions = []
        free_space_indices = np.argwhere(self.actual_map == 0)
        if len(free_space_indices) < self.number_of_bots:
            raise ValueError("Not enough free space to place all robots.")
        random_indices = np.random.choice(len(free_space_indices), self.number_of_bots, replace=False)
        for idx, i in enumerate(random_indices):
            position = tuple(free_space_indices[i])
            positions.append(position)
        return positions

    def update_explored_map_with_bots(self):
        for idx, pos in enumerate(self.bot_positions):
            y, x = pos
            self.explored_map[y, x] = 99 - idx  # Robot1:99, Robot2:98, Robot3:97, etc.

    # Rest of the code remains unchanged
    def get_random_position(self):
        free_positions = np.argwhere(self.actual_map == 0)
        idx = np.random.randint(len(free_positions))
        return tuple(free_positions[idx])

    def get_state(self):
        return self.explored_map.copy()

    def step(self, target_positions):
        prev_bot_positions = self.bot_positions.copy()
        rewards = [0.0 for _ in range(self.number_of_bots)]

        for idx, (bot_position, target_position) in enumerate(zip(self.bot_positions, target_positions)):
            x, y = target_position

            # Calculate Euclidean distance between current and target positions
            distance = np.linalg.norm(np.array([x, y]) - np.array(bot_position))
            distance = max(distance, 1.0)  # Prevent division by zero

            # Check for out of bounds
            if not (0 <= x < self.actual_map.shape[0] and 0 <= y < self.actual_map.shape[1]):
                rewards[idx] = PENALTY_OUT_OF_BOUNDS
                continue

            # Check if target position is invalid or known obstacle
            if self.actual_map[x, y] == -1 or self.explored_map[x, y] == 100:
                rewards[idx] = PENALTY_COLLISION
                continue

            # Store explored area before moving
            prev_explored = np.sum((self.explored_map == 0) | (self.explored_map == 100))

            # Move to the target position
            self.bot_positions[idx] = (x, y)
            self.update_map()

            # Calculate newly explored pixels
            new_explored = np.sum((self.explored_map == 0) | (self.explored_map == 100)) - prev_explored

            # If moved into an obstacle (discovered now)
            if self.actual_map[x, y] == 100:
                self.bot_positions[idx] = bot_position
                self.explored_map[x, y] = 100
                rewards[idx] = PENALTY_COLLISION
            else:
                # Reward is new explored pixels divided by distance
                rewards[idx] = REWARD_NEW_PIXEL * (new_explored / distance)

        # Penalize bots that are too close to each other
        for i in range(self.number_of_bots):
            for j in range(i + 1, self.number_of_bots):
                dist = np.linalg.norm(np.array(self.bot_positions[i]) - np.array(self.bot_positions[j]))
                if dist <= 5:
                    rewards[i] += -1000
                    rewards[j] += -1000
                elif dist <= 10:
                    rewards[i] += -100
                    rewards[j] += -100

        done = self.is_done()
        self.steps += 1
        return self.get_state(), rewards, done, prev_bot_positions

    def update_map(self):
        for bot_position in self.bot_positions:
            x, y = bot_position
            self.explored_map[x, y] = 5  # Bot's position
            angles = np.linspace(0, 2 * np.pi, 360)
            for angle in angles:
                for r in range(1, MAX_RANGE + 1):
                    nx = int(x + r * np.cos(angle))
                    ny = int(y + r * np.sin(angle))
                    if 0 <= nx < self.actual_map.shape[0] and 0 <= ny < self.actual_map.shape[1]:
                        if self.explored_map[nx, ny] == -1 and self.actual_map[nx, ny] != -1:
                            self.explored_map[nx, ny] = self.actual_map[nx, ny]
                        if self.actual_map[nx, ny] == 100:
                            break
                    else:
                        break

    def calculate_reward(self):
        # This function is no longer used since reward is calculated in the step function
        pass

    def is_done(self):
        # Ensure the polygon area is at least half of the max_area
        polygon_area = np.sum(self.mask)
        if polygon_area < 0.5 * self.max_area:
            return False  # Continue exploration until polygon area meets requirement

        # Check if all pixels inside the polygon are explored
        all_explored = np.all(self.explored_map[self.mask] != -1)
        
        # Check if the mapped area has reached the maximum area
        mapped_area = np.sum((self.explored_map == 0) | (self.explored_map == 100))
        
        # Terminate if any of the conditions are met
        if all_explored or mapped_area >= self.max_area or self.steps >= MAX_TIMESTEPS:
            return True
        return False

# Actor-Critic Model
class ActorCritic(nn.Module):
    def __init__(self, input_size, num_bots=NUMBER_OF_BOTS):
        super(ActorCritic, self).__init__()
        self.num_bots = num_bots
        self.fc = nn.Linear(input_size, 256)
        self.policy_head = nn.Linear(256, num_bots * 2)  # x and y for each bot
        self.value_head = nn.Linear(256, 1)
        self.env = None
        self.current_position = None

    def forward(self, x):
        x = torch.relu(self.fc(x))
        policy = self.policy_head(x)  # Shape: [batch_size, num_bots * 2]
        value = self.value_head(x)    # Shape: [batch_size, 1]
        return policy, value

    # def forward(self, x):
    #     x = torch.relu(self.fc(x))
    #     policies = []
    #     for _ in range(self.num_bots):
    #         logits = self.policy_head(x)

    #         # Mask invalid actions (unexplored, obstacles, or out of bounds)
    #         explored_free = (self.env.explored_map.flatten() == 0).astype(np.float32)
    #         mask = torch.tensor(explored_free, dtype=torch.float32)
    #         large_negative = -1e9  # Use a large negative value instead of -inf

    #         # Apply mask: invalid actions get a large negative value
    #         mask = torch.where(mask == 1, torch.tensor(0.0), torch.tensor(large_negative))

    #         policy_logits = logits + mask

    #         # Handle case where all actions are invalid
    #         if torch.all(mask == large_negative):
    #             # Assign uniform probabilities to allow at least some exploration
    #             policy = torch.softmax(logits, dim=-1)
    #         else:
    #             policy = torch.softmax(policy_logits, dim=-1)

    #         policies.append(policy)
    #     value = self.value_head(x)
    #     return policies, value

# Training parameters
NUM_EPISODES = 100
GAMMA = 0.99
LR = 1e-5

def main():
    num_bots = NUMBER_OF_BOTS  # Set the desired number of bots
    model = ActorCritic(input_size=MAP_SIZE * MAP_SIZE, num_bots=num_bots)
    optimizer = optim.Adam(model.parameters(), lr=LR)

    plt.ion()
    for episode in range(NUM_EPISODES):
        env = ExplorationEnv(number_of_bots=num_bots)
        state = env.get_state()
        state_flat = state.flatten()
        log_probs = []
        values = []
        rewards = []
        cumulative_reward = 0
        done = False
        step_count = 0

        while not done:
            step_count += 1
            state_tensor = torch.FloatTensor(state_flat)
            model.env = env
            policies, value = model(state_tensor)

            actions = []
            log_probs = []
            for policy in policies:
                dist = torch.distributions.Categorical(policy)
                action = dist.sample()
                log_prob = dist.log_prob(action)
                actions.append(action)
                log_probs.append(log_prob)

            # Convert actions to coordinates for each bot
            target_positions = [divmod(action.item(), env.actual_map.shape[1]) for action in actions]

            current_bot_positions = env.bot_positions
            next_state, rewards, done, prev_bot_positions = env.step(target_positions)
            next_bot_positions = env.bot_positions

            log_probs.extend(log_probs)
            values.append(value)
            rewards.extend(rewards)
            cumulative_reward += sum(rewards)
            state_flat = next_state.flatten()

            if step_count % 100 == 0:
                print(f"Episode {episode+1}, Step {step_count}, Cumulative Reward: {cumulative_reward}")

            # Visualization
            plt.clf()
            plt.subplot(1, 2, 1)
            plt.title("Actual Map")
            plt.imshow(env.actual_map, cmap='gray')
            for prev_pos, current_pos in zip(prev_bot_positions, next_bot_positions):
                plt.scatter(prev_pos[1], prev_pos[0], c='blue', marker='o', label='Previous')
                plt.scatter(current_pos[1], current_pos[0], c='red', marker='x', label='Current')
            plt.legend()

            plt.subplot(1, 2, 2)
            plt.title("Explored Map")
            cmap = colors.ListedColormap(['green', 'white', 'black'])
            bounds = [-1.5, -0.5, 0.5, 100.5]
            norm = colors.BoundaryNorm(bounds, cmap.N)
            display_map = np.copy(env.explored_map)
            display_map[display_map == 5] = 0  # Treat bot position as free space
            plt.imshow(display_map, cmap=cmap, norm=norm)
            for prev_pos, current_pos in zip(prev_bot_positions, next_bot_positions):
                plt.scatter(prev_pos[1], prev_pos[0], c='blue', marker='o')
                plt.scatter(current_pos[1], current_pos[0], c='red', marker='x')
            plt.legend(['Previous', 'Current'])

            plt.pause(0.01)

        # Compute returns and losses
        returns = []
        G = 0
        for reward in reversed(rewards):
            G = reward + GAMMA * G
            returns.insert(0, G)
        returns = torch.tensor(returns, dtype=torch.float32)  # Ensure float32
        loss = 0
        for log_prob, value, Gt in zip(log_probs, values, returns):
            advantage = Gt - value.item()
            policy_loss = -log_prob * advantage
            value_loss = nn.functional.mse_loss(value, torch.tensor([Gt], dtype=torch.float32).to(value.device))
            loss += policy_loss + value_loss
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(f"Episode {episode+1}/{NUM_EPISODES}, Total Reward: {sum(rewards)}")
        torch.save(model.state_dict(), 'model_weights.pth')

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    main()
