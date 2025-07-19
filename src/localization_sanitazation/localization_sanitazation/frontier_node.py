import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String
import numpy as np
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.mode_sub = self.create_subscription(String, '/robot_mode', self.mode_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.map_data = None
        self.robot_pose = None
        self.mode = "exploration"
        self.visited_goals = set()
        self.goal_active = False

        self.get_logger().info("Frontier Explorer Initialized.")

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def mode_callback(self, msg):
        self.mode = msg.data
        self.get_logger().info(f"Mode switched to: {self.mode}")

    def map_callback(self, msg):
        self.map_data = msg
        if self.robot_pose and not self.goal_active and self.nav_client.wait_for_server(timeout_sec=1.0):
            if self.mode in ["exploration", "sanitization"]:
                goal = self.find_reachable_frontier_goal()
            elif self.mode == "localization":
                goal = self.find_localization_goal()
            else:
                self.get_logger().warn(f"Unknown mode: {self.mode}")
                return

            if goal:
                point = (goal.pose.position.x, goal.pose.position.y)
                if not self.is_goal_visited(point):
                    self.visited_goals.add(point)
                    self.send_goal_to_nav2(goal)
                else:
                    self.get_logger().info("Skipping already-visited frontier goal.")
            else:
                self.get_logger().warn("No reachable frontier goal found.")

    def send_goal_to_nav2(self, goal_pose):
        self.goal_active = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.goal_active = False
            return
        self.get_logger().info("Goal accepted. Waiting...")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info("Goal completed.")
        self.goal_active = False

    def is_goal_visited(self, goal, threshold=0.5):
        return any(math.hypot(goal[0] - g[0], goal[1] - g[1]) < threshold for g in self.visited_goals)

    def find_reachable_frontier_goal(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        data = np.array(self.map_data.data).reshape((height, width))
        frontier_points = []

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y, x] == 0:  # Free space
                    neighbors = data[y-1:y+2, x-1:x+2].flatten()
                    if -1 in neighbors and all(n != 100 for n in neighbors):  # Adjacent unknown, not near wall
                        wx = origin.position.x + x * resolution
                        wy = origin.position.y + y * resolution
                        frontier_points.append((wx, wy))
                        self.get_logger().info(f"Evaluating frontier at ({wx:.2f}, {wy:.2f})")
        if not frontier_points:
            return None

        # Sort frontiers by distance from robot (descending to escape current room)
        rx, ry = self.robot_pose.position.x, self.robot_pose.position.y
        frontier_points.sort(key=lambda p: -math.hypot(rx - p[0], ry - p[1]))

        for wx, wy in frontier_points:
            if self.is_goal_visited((wx, wy)):
                continue
            goal = self.create_goal_pose(wx, wy)
            if self.check_path_exists(goal):
                return goal

        return None

    def check_path_exists(self, goal):
        if not self.path_client.wait_for_server(timeout_sec=1.0):
            return False
        path_goal = ComputePathToPose.Goal()
        path_goal.pose = goal
        path_goal.planner_id = ""
        future = self.path_client.send_goal_async(path_goal)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result.accepted if result else False

    def find_localization_goal(self):
        w = self.map_data.info.width
        h = self.map_data.info.height
        res = self.map_data.info.resolution
        ox = self.map_data.info.origin.position.x
        oy = self.map_data.info.origin.position.y

        wx = ox + (w // 2) * res
        wy = oy + (h // 2) * res
        return self.create_goal_pose(wx, wy)

    def create_goal_pose(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        return goal

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
