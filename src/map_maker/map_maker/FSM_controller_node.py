import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

# FSM States
TB3_INIT = 0
TB3_MOVING = 1
TB3_AVOIDING_OBSTACLE = 2
TB3_STOPPED = 3

class FSMController(Node):
    def __init__(self):
        super().__init__('FSM_Controller')

        # Publishers
        self.state_publisher = self.create_publisher(String, 'fsm_state', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, 'navigate_to_pose', 10)
        self.create_subscription(Bool, 'emergency_stop', self.emergency_stop_callback, 10)
        self.create_subscription(Bool, 'obstacle_detected', self.obstacle_detected_callback, 10)
        self.create_subscription(Bool, 'goal_reached', self.goal_reached_callback, 10)

        # Variables
        self.tb3_state = TB3_INIT
        self.emergency_stop = False
        self.obstacle_detected = False
        self.current_goal = PoseStamped()

        # Parameters for goal
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 3.0)
        self.set_goal()

    def set_goal(self):
        """Set and publish a new goal"""
        x = self.get_parameter('goal_x').value
        y = self.get_parameter('goal_y').value
        self.current_goal.pose.position.x = x
        self.current_goal.pose.position.y = y
        self.current_goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(self.current_goal)  # Publish goal
        self.get_logger().info(f"New goal published: x={x}, y={y}")

    def emergency_stop_callback(self, msg):
        """Handle emergency stop messages."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.tb3_state = TB3_STOPPED
            self.get_logger().warn("Emergency stop activated!")

    def obstacle_detected_callback(self, msg):
        """Handle obstacle detected messages."""
        self.obstacle_detected = msg.data
        if self.obstacle_detected and self.tb3_state == TB3_MOVING:
            self.tb3_state = TB3_AVOIDING_OBSTACLE
            self.get_logger().warn("Obstacle detected! Switching to avoiding obstacle state.")

    def goal_reached_callback(self, msg):
        """Handle goal reached messages."""
        if msg.data:
            self.get_logger().info("✅ Goal reached! Setting new one...")
            self.tb3_state = TB3_STOPPED
            time.sleep(1)  # Small delay before setting a new goal
            self.set_goal()  # Set a new goal automatically
            self.tb3_state = TB3_MOVING  # Start moving again



    def handle_init_state(self):
        """Handle TB3_INIT state."""
        self.get_logger().info("Initialization complete. Moving to goal...")
        self.tb3_state = TB3_MOVING

    def handle_moving_state(self):
        """Handle TB3_MOVING state."""
        if not self.obstacle_detected:
            self.get_logger().info("Navigating to goal...")
        else:
            self.tb3_state = TB3_AVOIDING_OBSTACLE

    def handle_avoiding_obstacle_state(self):
        """Handle TB3_AVOIDING_OBSTACLE state."""
        self.get_logger().info("Avoiding obstacle...")
        # Example avoidance strategy: Stop for a moment before re-evaluating
        rclpy.sleep(1.0)  # Pause to simulate avoidance action
        self.obstacle_detected = False
        self.tb3_state = TB3_MOVING

    def handle_stopped_state(self):
        """Handle TB3_STOPPED state."""
        self.get_logger().info("Robot stopped.")
        # Additional logic for stopping if needed

    def control_loop(self):
        """Main control loop for the FSM."""
        self.get_logger().info("Starting FSM Controller")

        while rclpy.ok():
            rclpy.spin_once(self)

            if self.emergency_stop:
                self.tb3_state = TB3_STOPPED

            # FSM Logic
            if self.tb3_state == TB3_INIT:
                self.handle_init_state()

            elif self.tb3_state == TB3_MOVING:
                self.handle_moving_state()

            elif self.tb3_state == TB3_AVOIDING_OBSTACLE:
                self.handle_avoiding_obstacle_state()

            elif self.tb3_state == TB3_STOPPED:
                self.handle_stopped_state()

def main(args=None):
    rclpy.init(args=args)
    controller = FSMController()
    controller.control_loop()
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
