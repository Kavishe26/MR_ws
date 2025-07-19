from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist,PoseArray
from sensor_msgs.msg import LaserScan
import numpy as np
import heapq , math , random , yaml
import scipy.interpolate as si
import sys , threading , time , rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String



with open("/home/kavishe/ros2_ws/src/map_maker/config/params.yaml", 'r') as file:
    param = yaml.load(file, Loader=yaml.FullLoader)

lookahead_distance = param["lookahead_distance"]  
speed = param["speed"]
expansion_size = param["expansion_size"]
target_error = param["target_error"]
robot_r = param["robot_r"]
MAX_STUCK_COUNT = 800
PROGRESS_THRESHOLD = 0.1  
pathGlobal = 0
XGoal = 0
YGoal = 0
###################
#Computes the yaw (rotation around the z-axis) for robot orientation.
###################
def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


#################
# A* Algorithm
    # Steps:

    # Define neighbors (adjacent cells, including diagonals).
    # Calculate cost functions gscore (actual cost) and fscore (estimated total cost).
    # Backtrack from the goal to reconstruct the path if reachable.
#################

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    # Define possible neighbor positions (up, down, left, right, and diagonals)
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    # Set to keep track of visited nodes
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

###################
# Smooth Trajectory
  #Steps:

    #Uses scipy to interpolate x and y coordinates.
    #Returns a smooth trajectory.
###################
def bspline_planning(array, sn):
    try:
        # Convert the input array to numpy array
        array = np.array(array)
        # Extract x and y coordinates from the input array
        x = array[:, 0]
        y = array[:, 1]
        # N is the degree of the B-spline (2 for quadratic B-spline)
        N = 2
        # t is the parameterization of the input points
        t = range(len(x))

        # Compute B-spline representation for x and y coordinates
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        # List representation of B-spline coefficients for x and y
        x_list = list(x_tup)
        xl = x.tolist()
        # Extend the coefficient lists to add extra points for smoothing
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        # If an exception occurs, return the original array
        path = array
    return path

########################
# pure pursuit algorithm
   #Implements the pure pursuit algorithm for path following.
   #Steps:

    #Finds the closest point on the path ahead of the robot.
    #Calculates the desired steering angle and adjusts velocity.
########################

def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

##########################
# Identify frontier points
  #Identifies frontier points (boundary between explored and unexplored areas) in the map.
  #occupied cell = 100
  #not occupied cell = 0
  #unexplored cell = -1
  #frontier = 2
##########################
def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
    return matrix

#################################
# Assign frontier point to groups
 #Groups frontier points based on connectivity using depth-first search (DFS).
#################################
def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

#################################
# Depth-first search (DFS)
  #consider only cells with value = 2 and Recursively call the function for all 8 neighboring cells (including diagonals) to identify and assign connected frontier points to the same group.
#################################
def dfs(matrix, i, j, group, groups):
    """
    Depth-first search (DFS) to explore and assign group numbers to connected frontier points.
    """
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups) # bottom right diagonal
    dfs(matrix, i - 1, j - 1, group, groups) # top left diagonal
    dfs(matrix, i - 1, j + 1, group, groups) # top right diagonal
    dfs(matrix, i + 1, j - 1, group, groups) # bottom left diagonal
    return group + 1


###########################
#Filters and selects the top 5 largest frontier groups with more than 10 points.
###########################
def fGroups(groups):
    # Sort the groups based on the length of their points (descending order)
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    # Filter the top five groups with more than 2 points
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 10]    
    return top_five_groups

#############################################
#Computes the centroid of a group of coordinates.
#############################################
def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)     # Number of points
    sum_x = sum(x_coords) # Sum of x coordinates
    sum_y = sum(y_coords) # Sum of y coordinates
    mean_x = sum_x / n    # Mean of x coordinates
    mean_y = sum_y / n    # Mean of y coordinates
    centroid = (int(mean_x), int(mean_y))  # Centroid as a tuple of integers
    return centroid


########################
# Finds the closest frontier group and calculates a path to it using A*
  #Steps:

    #Calculates centroids of groups.
    #Evaluates paths using a scoring system (group size vs. distance).
    #Returns the path to the best-scored group.
#########################
def findClosestGroup(matrix,groups, current,resolution,originX,originY):
    targetP = None
    distances = []
    paths = []
    score = []
    max_score = -1 #max score index
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        path = astar(matrix, current, middle)
        path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
        total_distance = pathLength(path)
        distances.append(total_distance)
        paths.append(path)
    for i in range(len(distances)):
        if distances[i] == 0:
            score.append(0)
        else:
            score.append(len(groups[i][1])/distances[i])
    for i in range(len(distances)):
        if distances[i] > target_error*3:
            if max_score == -1 or score[i] > score[max_score]:
                max_score = i
    if max_score != -1:
        targetP = paths[max_score]
    else: # if groups are closer than target_error*2, select a random point as the target. This helps the robot in some situations.
        index = random.randint(0,len(groups)-1)
        target = groups[index][1]
        target = target[random.randint(0,len(target)-1)]
        path = astar(matrix, current, target)
        targetP = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
    return targetP

################################
#Computes the total length of a path.
################################
def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

####################
# Update the costmap to Expands obstacles in the map to account for the robot's size and safety margins.
####################
def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

#####################################
#Main exploration algorithm.
 #Steps:

    #Updates the map with expanded obstacles.
    #Identifies and groups frontier points.
    #Finds the closest group and plans a path.
    #determine Smooth path using B-spline.

    #Output: Updates pathGlobal with the planned path.
#####################################
def exploration(data,width,height,resolution,column,row,originX,originY):
        global pathGlobal #a global path variable
        # Step 1: Expand obstacles in the map
        data = costmap(data,width,height,resolution)

        # Step 2: Set the robot's current position in the map 
        data[row][column] = 0 

        # Step 3: Mark certain obstacles, considering a threshold (5 in this case)
        data[data > 5] = 1  

        # Step 4: Find frontier points in the map
        data = frontierB(data) 

        # Step 5: Assign frontier points to groups based on spatial connectivity
        data,groups = assign_groups(data) 

        # Step 6: Sort groups from small to large and take the largest groups
        groups = fGroups(groups)

        # Step 7: Find the closest group and calculate a path to it using A*
        current = (row, column)
        path = findClosestGroup(data, groups, current, resolution, originX, originY)

        # Step 8: Generate a smooth path using B-spline
        if path:
            path = bspline_planning(path, sn=100)
        else:
            pathGlobal = []
            

        # Step 9: Update the global path variable
        pathGlobal = path




###########################
#ROS2 Node for managing exploration and navigation
###########################
class Explorer(Node):
    def __init__(self):
        #step 0: Initialization of ROS2 node, subscriptions, and publisher
        super().__init__('explorer')
        self.subscription = self.create_subscription(OccupancyGrid,'map',self.map_callback,10)
        self.subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.subscription = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        self.subscription = self.create_subscription(String,'status',self.status_callback,10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_frontier = self.create_publisher(Pose, 'frontier', 10)
        self.publisher_path = self.create_publisher(PoseArray, 'path', 10)
        self.explore_status = self.create_publisher(String, 'exploration', 10)
        print("[INFO] EXPLORATION MODE ACTIVE")
        self.kesif = True
        # self.status_msg = None
        self.status_received = False
        self.lock = threading.Lock()
        # step 1: Start exploration thread
        threading.Thread(target=self.exp).start() 


    #############################
    # Main exploration loop.
       #Executes exploration steps, including map updates, path planning, and goal checking.
       #Publishes the planned path and exploration status.
         #waiting for data
         #EXPLORATION COMPLETED
         #NEW TARGET SET
         #TARGET REACHED
         #TARGET REJECTED
    #############################      

    def exp(self):
        msg_f = Pose()
        stuck_count = 0  # Counter to track getting stuck

        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
                self.get_logger().info("Waiting for sensor data...")
                time.sleep(0.1)
                continue

            if self.kesif:  # If exploration mode is active
                if isinstance(pathGlobal, int) and pathGlobal == 0:
                    column = int((self.x - self.originX) / self.resolution)
                    row = int((self.y - self.originY) / self.resolution)
                    self.get_logger().info("Constructing map and planning new path...")
                    
                    exploration(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)
                    self.path = pathGlobal
                else:
                    self.path = pathGlobal

                if isinstance(self.path, int) and self.path == -1:
                    self.get_logger().info("🚀 EXPLORATION COMPLETED!")
                    status_msg = String()
                    status_msg.data = "EXPLORATION COMPLETED"
                    self.explore_status.publish(status_msg)
                    return  # Exit loop once exploration is done

                # Set new target automatically
                self.c = int((self.path[-1][0] - self.originX) / self.resolution)
                self.r = int((self.path[-1][1] - self.originY) / self.resolution)

                with self.lock:
                    self.kesif = False  # Mark exploration as in progress

                self.i = 0
                stuck_count = 0
                self.get_logger().info("[INFO] NEW TARGET SET")
                
                t = pathLength(self.path) / speed
                t -= 0.2  # Slight buffer
                self.t = threading.Timer(t, self.target_callback)  # Start the next goal automatically
                self.t.start()


    '''
    def exp(self):
        msg_f = Pose()
        twist = Twist()
        stuck_count = 0  # Counter to keep track of being stuck
        last_distance_to_target = float('inf')  # Initialize with a large value


        while True:
            # Wait for sensor data
            if not hasattr(self, 'map_data') or not hasattr(self, 'odom_data') or not hasattr(self, 'scan_data'):
                self.get_logger().info("Waiting for data...")
                time.sleep(0.1)
                continue

            if self.kesif:
                ###################
                # Exploration mode:
                ###################
                if isinstance(pathGlobal, int) and pathGlobal == 0:
                    # Step 1: Construct the map and plan a new path
                    column = int((self.x - self.originX) / self.resolution)
                    row = int((self.y - self.originY) / self.resolution)
                    self.get_logger().info("[INFO] Constructing map and planning new path...")
                    exploration(self.data, self.width, self.height, self.resolution, column, row, self.originX, self.originY)
                    self.path = pathGlobal
                else:
                    # Step 2: Following a planned path
                    self.path = pathGlobal
                if isinstance(self.path, int) and self.path == -1:
                    print("[INFO] EXPLORATION COMPLETED")
                    status_msg = String()
                    status_msg.data = "EXPLORATION COMPLETED"
                    self.explore_status.publish(status_msg)
                    sys.exit()

                # Step 3: Set new target and start a timer
                self.c = int((self.path[-1][0] - self.originX) / self.resolution)
                self.r = int((self.path[-1][1] - self.originY) / self.resolution)

                # Use the lock
                with self.lock:
                  self.kesif = False

                self.i = 0
                stuck_count = 0  # Reset the stuck counter
                last_distance_to_target = float('inf')  # Reset the last distance to target
                print("[INFO] NEW TARGET SET")
                t = pathLength(self.path) / speed
                t = t - 0.2  # Subtract 0.2 seconds from the time calculated according to the formula x = v * t. The exploration function is called after t seconds.
                self.t = threading.Timer(t, self.target_callback)  # Start a timer to trigger the target callback
                self.t.start()

            else:
                msg_f.position.x = self.path[-1][0]
                msg_f.position.y = self.path[-1][1]
                # Print the values using f-strings
                self.get_logger().info(f"x = {msg_f.position.x}, y = {msg_f.position.y}")
                self.publisher_frontier.publish(msg_f)

                path_msg = PoseArray()
                # Populate the poses in the path_msg
                for point in self.path:
                    pose = Pose()
                    pose.position.x = point[0]
                    pose.position.y = point[1]
                    pose.orientation.w = 1.0  # Set the orientation as needed
                    path_msg.poses.append(pose)
                self.publisher_path.publish(path_msg)
                   
            

                # Step 5: Check if the target is reached
                distance_to_target = ((self.x - self.path[-1][0]) ** 2 + (self.y - self.path[-1][1]) ** 2) ** 0.5
                if distance_to_target < target_error:
                    with self.lock:
                        self.kesif = True
                    self.get_logger().info("[INFO] TARGET REACHED")
                    time.sleep(18)  # Wait after target is reached
                    self.t.join()  # Wait until the timer finishes.

                elif self.status_received == True:
                    # If the status is rejected, reset exploration mode
                    self.status_received = False
                    with self.lock:
                        self.kesif = True
                    self.get_logger().info("[INFO] TARGET REJECTED")
            # Route Tracking Block End
    '''
    #######################
    #Plans the next target after reaching the current one.
    #######################
    def target_callback(self):
        self.get_logger().info("[INFO] Planning next target...")
        exploration(self.data,self.width,self.height,self.resolution,self.c,self.r,self.originX,self.originY)

    ######################
    #Processes laser scan data. 
    ######################   
    def scan_callback(self,msg):
        self.scan_data = msg
        self.scan = msg.ranges

    ######################
    #Processes the occupancy grid map.
    #####################
    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data
    
    ########################    
    #Updates the robot's odometry data.
    ########################
    def odom_callback(self,msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    
    
    ##########################    
    #Handles navigation status updates,come from the path_planning_navigator_node.py, where they are published using the target_status publisher based on the Nav2 goal outcomes. 
    ##########################   
    def status_callback(self,msg):
        self.status_msg = msg
        self.get_logger().info(f"Received status: {self.status_msg.data}")
        self.status_received = True
        



###############
# Main function
###############
def main(args=None):
   
    rclpy.init(args=args)
    explorer = Explorer()

    # Start the exploration automatically
    explorer.get_logger().info("🛠 Starting autonomous exploration...")
    explorer.exp()  # Call exploration directly

    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
