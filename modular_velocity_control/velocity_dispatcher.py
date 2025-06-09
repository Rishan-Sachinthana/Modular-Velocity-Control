# velocity_dispatcher.py (handles velocity and publishes positions)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import yaml
import os
import math
import json
from ament_index_python.packages import get_package_share_directory

class VelocityDispatcher(Node):
    def __init__(self):
        super().__init__('velocity_dispatcher')

        config_path = os.path.join(
            get_package_share_directory('modular_velocity_control'),
            'config', 'formation.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.matrix = config['matrix']
        self.orientation = config['orientation']
        self.pivot = config.get('pivot', self.compute_centroid())

        self.robots = self.get_robot_list()

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.robot_publishers = {
            f'robot{i+1}': self.create_publisher(Twist, f'/robot{i+1}/cmd_vel', 10)
            for i in range(len(self.robots))
        }

        self.global_theta = 0.0
        self.structure_position = [0.0, 0.0]

        self.position_pub = self.create_publisher(String, '/robot_positions', 10)
        self.positions = self.compute_initial_positions()

        self.last_cmd_vel = Twist()
        self.timer = self.create_timer(0.1, self.update_motion)

        self.get_logger().info('Velocity Dispatcher Initialized')

    def compute_centroid(self):
        positions = [(r, c) for r, row in enumerate(self.matrix) for c, val in enumerate(row) if val]
        x = sum(p[0] for p in positions) / len(positions)
        y = sum(p[1] for p in positions) / len(positions)
        return [int(round(x)), int(round(y))]

    def get_robot_list(self):
        return [f'robot{i+1}' for i in range(sum(val for row in self.matrix for val in row))]

    def compute_initial_positions(self):
        pos = {}
        index = 0
        for r, row in enumerate(self.matrix):
            for c, val in enumerate(row):
                if val:
                    dx = c - self.pivot[1]
                    dy = r - self.pivot[0]
                    local_theta = math.radians(self.orientation[r][c])
                    pos[f'robot{index+1}'] = {
                        'dx': dx,
                        'dy': -dy,
                        'local_theta': local_theta
                    }
                    index += 1
        return pos

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg

    def update_motion(self):
        msg = self.last_cmd_vel

        self.global_theta += msg.angular.z * 0.1

        vx = msg.linear.x * math.cos(self.global_theta) - msg.linear.y * math.sin(self.global_theta)
        vy = msg.linear.x * math.sin(self.global_theta) + msg.linear.y * math.cos(self.global_theta)

        self.structure_position[0] += vx * 0.1
        self.structure_position[1] += vy * 0.1

        current_positions = {}
        for name, data in self.positions.items():
            dx = data['dx']
            dy = data['dy']
            local_theta = data['local_theta']

            rel_x = dx * math.cos(self.global_theta) - dy * math.sin(self.global_theta)
            rel_y = dx * math.sin(self.global_theta) + dy * math.cos(self.global_theta)

            gx = self.structure_position[0] + rel_x
            gy = self.structure_position[1] + rel_y
            current_positions[name] = [gx, gy, self.global_theta + local_theta]

            # Compute twist relative to pivot
            local_twist = Twist()
            local_twist.linear.x = msg.linear.x - msg.angular.z * rel_y
            local_twist.linear.y = msg.linear.y + msg.angular.z * rel_x
            local_twist.angular.z = msg.angular.z

            # Adjust twist to robot's local orientation
            cos_o = math.cos(local_theta)
            sin_o = math.sin(local_theta)

            adjusted_x = local_twist.linear.x * cos_o + local_twist.linear.y * sin_o
            adjusted_y = -local_twist.linear.x * sin_o + local_twist.linear.y * cos_o

            local_twist.linear.x = adjusted_x
            local_twist.linear.y = adjusted_y

            self.robot_publishers[name].publish(local_twist)

        self.publish_positions(current_positions)

    def publish_positions(self, pos_dict):
        msg = String()
        msg.data = json.dumps(pos_dict)
        self.position_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = VelocityDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
