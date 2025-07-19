import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_srvs.srv import Empty
from std_msgs.msg import Bool, Header
import time


class Localization(Node):
    def __init__(self):
        super().__init__('localization')

        # Publishers
        self.localization_complete_pub = self.create_publisher(Bool, 'localization_complete', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        amcl_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.amcl_pose_callback, amcl_qos)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Service to reinitialize localization
        self.reinit_global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.reinit_global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reinitialize_global_localization service...')

        # Variables
        self.initial_pose = None
        self.x_cov, self.y_cov, self.z_cov = float('inf'), float('inf'), float('inf')
        self.localization_done = False
        self.moving = False
        self.move_state = 0  # Track which movement phase

    def odometry_callback(self, msg):
        """Updates the initial pose from odometry data."""
        self.initial_pose = msg.pose.pose

    def amcl_pose_callback(self, msg):
        """Updates covariance values from AMCL pose data."""
        self.x_cov = msg.pose.covariance[0]  # Covariance in X
        self.y_cov = msg.pose.covariance[7]  # Covariance in Y
        self.z_cov = msg.pose.covariance[35] # Covariance in Yaw

        self.get_logger().info(f"Current Covariance - X: {self.x_cov:.2f}, Y: {self.y_cov:.2f}, Z: {self.z_cov:.2f}")

        # Check if covariance is within threshold
        if self.x_cov < 1.5 and self.y_cov < 1.5 and self.z_cov < 0.8:
            self.localization_done = True
            self.stop_robot()
            self.localization_complete_pub.publish(Bool(data=True))
            self.get_logger().info('Localization complete!')

    def move_robot(self):
        """Move the robot in larger patterns for full exploration."""
        vel_msg = Twist()

        # Define movement phases for exploration
        if self.move_state == 0:
            self.get_logger().info("Moving straight ahead.")
            vel_msg.linear.x = 0.3  # Move straight
            vel_msg.angular.z = 0.0
        elif self.move_state == 1:
            self.get_logger().info("Turning to cover more area.")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.3  # Turn to explore another direction
        elif self.move_state == 2:
            self.get_logger().info("Moving straight again.")
            vel_msg.linear.x = 0.3
            vel_msg.angular.z = 0.0
        elif self.move_state == 3:
            self.get_logger().info("Final rotation to scan surroundings.")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.3

        self.velocity_publisher.publish(vel_msg)
        time.sleep(3)  # Move for 3 seconds
        self.move_state = (self.move_state + 1) % 4  # Cycle phases

    def stop_robot(self):
        """Stops the robot gracefully after localization."""
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        self.get_logger().info("Robot stopped.")

    def spin_loop(self):
        """Main loop to handle continuous movement during localization."""
        self.get_logger().info("Waiting for initial odometry...")
        time.sleep(2)  # Wait to ensure odometry is stable

        self.get_logger().info("Starting exploration for localization.")
        self.moving = True

        # Move across multiple directions for exploration
        while rclpy.ok() and not self.localization_done:
            rclpy.spin_once(self)
            self.move_robot()  # Move robot in different patterns


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    node.spin_loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
