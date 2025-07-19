import sys
import rclpy
import math
import numpy as np
from copy import deepcopy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray

class SanitizationNode(Node):
    def __init__(self):
        super().__init__('sanitization_node')

        self.map_data = None
        self.robot_pose = None
        self.sanitized_map = None

        self.localization_done = False
        self.navigation_done = False
        self.timer_started = False

        # Publishers
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        self.sanitization_status_publisher = self.create_publisher(Bool, 'sanitization_status', 10)
        self.sanitized_map_publisher = self.create_publisher(OccupancyGrid, 'sanitized_map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sanitized_markers', 10)

        # Subscribers
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_callback, 10)
        self.create_subscription(Bool, 'navigation_complete', self.navigation_callback, 10)
        self.create_subscription(Bool, 'localization_complete', self.localization_callback, 10)

        self.get_logger().info("Sanitization node initialized. Waiting for localization and navigation completion...")

    def localization_callback(self, msg):
        if msg.data and not self.localization_done:
            self.localization_done = True
            self.get_logger().info("Received localization completion flag.")
            self.check_and_start_sanitization()

    def navigation_callback(self, msg):
        if msg.data and not self.navigation_done:
            self.navigation_done = True
            self.get_logger().info("Received navigation completion flag.")
            self.check_and_start_sanitization()

    def check_and_start_sanitization(self):
        if self.localization_done and self.navigation_done and not self.timer_started:
            self.timer_started = True
            self.get_logger().info("Localization and navigation complete. Starting sanitization.")
            self.mode_pub.publish(String(data="sanitization"))
            self.sanitization_status_publisher.publish(Bool(data=True))
            self.timer = self.create_timer(0.1, self.sanitization_loop)

    def map_callback(self, msg):
        self.map_data = msg
        self.sanitized_map = deepcopy(msg)  # Create editable copy
        self.get_logger().info("Map data received.")

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def sanitization_loop(self):
        if self.robot_pose and self.sanitized_map:
            x = self.robot_pose.position.x
            y = self.robot_pose.position.y
            self.update_sanitized_map(x, y)

    def update_sanitized_map(self, x, y):
        info = self.sanitized_map.info
        width = info.width
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        i = int((x - origin_x) / resolution)
        j = int((y - origin_y) / resolution)
        index = j * width + i

        if 0 <= index < len(self.sanitized_map.data):
            data = list(self.sanitized_map.data)
            if data[index] != 50:
                data[index] = 50  # Mark as sanitized
                self.sanitized_map.data = tuple(data)
                self.sanitized_map_publisher.publish(self.sanitized_map)
                self.publish_sanitization_marker(i, j, resolution, origin_x, origin_y)
    def publish_sanitization_marker(self, i, j, resolution, origin_x, origin_y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sanitized_cells"
        marker.id = i * 1000 + j  # unique ID per cell
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = resolution
        marker.scale.y = resolution
        marker.scale.z = 0.01  # flat marker
    
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.6  # semi-transparent
    
        marker.pose.position.x = origin_x + (i + 0.5) * resolution
        marker.pose.position.y = origin_y + (j + 0.5) * resolution
        marker.pose.position.z = 0.0
    
        marker.pose.orientation.w = 1.0
    
        marker_arr = MarkerArray()
        marker_arr.markers.append(marker)
        self.marker_pub.publish(marker_arr)

def main():
    rclpy.init()
    node = SanitizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
