import itertools
import random
import yaml
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose, FollowPath
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator

class Navigator(Node):
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals), 
        'random': lambda goals: (random.shuffle(goals), itertools.cycle(goals))[1],
    }

    def __init__(self):
        super().__init__('navigator')

        self.route = []
        self.current_goal = NavigateToPose.Goal()
        self.localization_complete = False

        # Subscribers
        self.create_subscription(Bool, 'localization_complete', self.bool_callback, 10)

        # Action clients
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.client.wait_for_server()

        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.follow_path_client.wait_for_server()

        # Load route from file
        route_file_path = r"/home/kavishe/ros2_ws/src/localization_sanitazation/localization_sanitazation/navgoals.yaml"
        with open(route_file_path, 'r') as f:
            route_yaml = yaml.safe_load(f)

        self.route_mode = route_yaml.get('mode', 'inorder')
        if self.route_mode not in Navigator.route_modes:
            self.get_logger().error(f"Unknown route mode '{self.route_mode}'. Exiting Navigator.")
            sys.exit(1)

        poses = route_yaml.get('poses', [])
        if not poses:
            self.get_logger().info("Navigator initialized with no goals")
            sys.exit(1)

        self.goals = Navigator.route_modes[self.route_mode](poses)
        self.length = len(poses)
        self.number_of_goals = 0
        self.get_logger().info(f"Navigator initialized with {self.length} goals in {self.route_mode} mode")

        # Publisher to notify navigation completion
        self.navigation_complete_publisher = self.create_publisher(Bool, 'navigation_complete', 10)

    def to_move_goal(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def route_forever(self):
        self.get_logger().info(f"Route mode is '{self.route_mode}', getting next goal")
        try:
            current_goal = self.to_move_goal(next(self.goals))
        except StopIteration:
            self.get_logger().info("All goals visited, Stopping Navigator")
            return

        self.number_of_goals += 1
        self.get_logger().info(f"Sending target goal: ({current_goal.pose.pose.position.x}, {current_goal.pose.pose.position.y})")
        self._send_goal_future = self.client.send_goal_async(
            current_goal,
            feedback_callback=self.feedback_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        if self.number_of_goals < self.length:
            self.route_forever()
        else:
            self.get_logger().info("No more goals, Stopping Navigator")
            # Publish navigation completion signal
            msg = Bool()
            msg.data = True
            self.navigation_complete_publisher.publish(msg)
            self.get_logger().info("Navigation complete, Published completion signal")

    def feedback_callback(self, feedback_msg):
        pass

    def bool_callback(self, msg):
        self.localization_complete = msg.data
        if self.localization_complete:
            self.get_logger().info("Localization complete, Starting Navigator")
            self.route_forever()

def main():
    rclpy.init()
    try:
        navigator = Navigator()
        navigator.get_logger().info("Waiting for localization to complete")

        # Let the subscriber trigger navigation start
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        pass
    except BaseException as e:
        print(f'Exception in navigator: {e}', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
