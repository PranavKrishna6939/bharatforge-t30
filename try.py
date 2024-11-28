#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import sys

class SingleBotNavigator(Node):
    def __init__(self):
        super().__init__('single_bot_navigator')

        # Parameters
        self.declare_parameter('robot_namespace', '')  # e.g., '/robot1'
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Action server name+namespace
        if self.robot_namespace:
            self.action_server_name = f'{self.robot_namespace}/navigate_to_pose'
        else:
            self.action_server_name = 'navigate_to_pose'
        
        self.get_logger().info(f'Using action server: {self.action_server_name}')

    def send_goal(self, x, y, theta):
        # Convert theta from degrees to radians
        theta_rad = theta * (3.14159265 / 180.0)
        q = quaternion_from_euler(0, 0, theta_rad)

        # Construct the command
        command = [
            'ros2', 'action', 'send_goal', self.action_server_name, 'nav2_msgs/action/NavigateToPose',
            f'{{pose: {{header: {{frame_id: "map"}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: {q[0]}, y: {q[1]}, z: {q[2]}, w: {q[3]}}}}}}}}'
        ]

        self.get_logger().info(f'Sending goal to navigate to the specified pose: x={x}, y={y}, theta={theta} degrees')

        # Execute the command
        result = subprocess.run(command, capture_output=True, text=True)

        if result.returncode == 0:
            self.get_logger().info('Goal sent successfully.')
            self.get_logger().info(result.stdout)
        else:
            self.get_logger().error('Failed to send goal.')
            self.get_logger().error(result.stderr)

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    """
    import math
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

def get_user_input():
    """
    Prompts the user to input target coordinates and orientation.

    Returns:
        tuple: (x, y, theta)
    """
    try:
        x = float(input("Enter target X coordinate: "))
        y = float(input("Enter target Y coordinate: "))
        theta = float(input("Enter target orientation (theta in degrees, default 0): ") or 0)
    except ValueError:
        print("Invalid input. Please enter numeric values.")
        sys.exit(1)

    return x, y, theta

def main(args=None):
    rclpy.init(args=args)

    navigator = SingleBotNavigator()

    x, y, theta = get_user_input()

    navigator.send_goal(x, y, theta)

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation interrupted by user.')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()