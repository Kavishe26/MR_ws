import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import math


def create_quaternion_from_yaw(yaw):
    """Helper function to create a quaternion from a yaw angle."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class NavigatorWithRotation(Node):
    def __init__(self):
        super().__init__('navigator_with_rotation')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.rotation_speed = 0.5  # radians/second
        self.rotation_duration_per_radian = 2.0

        self.goal_pose = PoseStamped()
        self.initialize_goal()

    def initialize_goal(self):
        self.goal_pose.pose.position.x = 3.0
        self.goal_pose.pose.position.y = 3.0
        self.goal_pose.pose.orientation = create_quaternion_from_yaw(math.pi / 4)

        self.get_logger().info(f"Default goal set: x={self.goal_pose.pose.position.x}, y={self.goal_pose.pose.position.y}")

    def move_robot(self, linear_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        self.cmd_vel_pub.publish(twist_msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def rotate(self, angle_radians):
        self.get_logger().info(f"Rotating by {math.degrees(angle_radians):.2f} degrees.")

        duration = abs(angle_radians) * self.rotation_duration_per_radian
        angular_velocity = self.rotation_speed if angle_radians > 0 else -self.rotation_speed

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).seconds < duration:
            twist_msg = Twist()
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            rclpy.spin_once(self)

        self.stop_robot()
        self.get_logger().info("Rotation complete.")

    def go_to_pose(self, pose):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info("Sending navigation goal.")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached successfully.")
            return True
        else:
            self.get_logger().warning("Navigation failed. Retrying...")
            pose.pose.position.x += 0.1
            pose.pose.position.y += 0.1
            return self.retry_pose(pose)

    def retry_pose(self, pose):
        if self.go_to_pose(pose):
            return True
        self.get_logger().error("Retry failed. Stopping.")
        self.stop_robot()
        return False

    def feedback_callback(self, feedback_msg):
        current_pose = feedback_msg.feedback.current_pose.pose
        self.get_logger().info(f"Feedback - Position: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")

    def main_loop(self):
        if self.go_to_pose(self.goal_pose):
            self.rotate(math.radians(90))
        else:
            self.get_logger().error("Navigation unsuccessful.")
            self.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    navigator = NavigatorWithRotation()
    navigator.main_loop()
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

