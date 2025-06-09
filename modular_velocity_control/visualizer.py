# visualizer.py (subscribes to position updates and publishes marker array)

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
import json
import math

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')

        self.positions = {}
        self.subscription = self.create_subscription(
            String,
            '/robot_positions',
            self.position_callback,
            10
        )

        self.publisher = self.create_publisher(MarkerArray, '/robot_states', 10)
        self.timer = self.create_timer(0.2, self.publish_markers)

    def position_callback(self, msg):
        self.positions = json.loads(msg.data)

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, (name, pos) in enumerate(self.positions.items()):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'robots'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.1
            marker.pose.orientation.z = math.sin(pos[2] / 2)
            marker.pose.orientation.w = math.cos(pos[2] / 2)
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.2 * i
            marker.color.g = 0.5
            marker.color.b = 1.0 - 0.2 * i
            marker_array.markers.append(marker)

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
