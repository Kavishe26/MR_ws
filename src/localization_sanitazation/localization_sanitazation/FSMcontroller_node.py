import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.4
MIN_SAFE_DISTANCE = 0.6  
SIDE_DISTANCE_THRESHOLD = 0.3 

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        
        # ROS2 Publishers & Subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Initialize laser scan data
        self.laser_data = None

    def laser_callback(self, msg):
        """Receives laser scan data and makes movement decisions."""
        self.laser_data = msg
        self.explore_environment()

    def explore_environment(self):
        if self.laser_data is None:
            return

        # Extract distances
        ranges = self.laser_data.ranges
        front = min(min(ranges[-10:] + ranges[:10]), 10)
        left = min(ranges[60:100])
        right = min(ranges[260:300])

        vel_msg = Twist()

        if front < MIN_SAFE_DISTANCE:
            # Blocked ahead: choose direction with more space
            if left > right and left > SIDE_DISTANCE_THRESHOLD:
                vel_msg.angular.z = ANGULAR_SPEED
                self.get_logger().info("Turning left, blocked ahead.")
            elif right > left and right > SIDE_DISTANCE_THRESHOLD:
                vel_msg.angular.z = -ANGULAR_SPEED
                self.get_logger().info("Turning right, blocked ahead.")
            else:
                # No good side → rotate slowly to search
                vel_msg.angular.z = ANGULAR_SPEED / 2
                self.get_logger().info("Rotating to search for exit...")
        
        elif left < SIDE_DISTANCE_THRESHOLD:
            vel_msg.angular.z = -ANGULAR_SPEED / 2
            self.get_logger().info("Avoiding left wall...")

        elif right < SIDE_DISTANCE_THRESHOLD:
            vel_msg.angular.z = ANGULAR_SPEED / 2
            self.get_logger().info("Avoiding right wall...")

        else:
            vel_msg.linear.x = LINEAR_SPEED
            self.get_logger().info("Moving forward...")

        self.velocity_publisher.publish(vel_msg)


def main():
    rclpy.init()
    explorer = Explorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
