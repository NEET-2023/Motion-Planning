import rospy
import os
import cv2
import numpy as np
import skimage.measure
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from node import Node


class Navigator():
    def __init__(self, occupancy_grid, xmin, xmax, ymin, ymax, waypoints):
        # debugging parameter
        self.debug = False
        self.debug_location = np.array([0, 0, 0])
        # Map inforation intializations
        self.occupancy_grid = occupancy_grid
        self.ground_dist = 0
        self.flight_height = 10
        self.max_row, self.max_col = np.array(self.occupancy_grid.shape) - 1
        self.min_x, self.max_x = xmin, xmax
        self.min_y, self.max_y = ymin, ymax
        self.max_z = None 
        # Tracking of which waypoint/point in path we are following
        self.waypoints = waypoints 
        self.waypoint_index = 0
        self.next_waypoint = True
        self.done_travelling = False
        self.path_plan = True
        self.path_found = True
        self.path = None 
        self.path_index = 0
        # PD controller initializations
        self.kp = 1.0
        self.kd = 0
        self.max_v = 1
        self.prev_v = np.array([0, 0])
        self.below_height_threshold = False
        # May or may not control z independently. TBD
        self.max_vz = 1
        self.prev_vz = 0
        # ROS relevant initializations
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
        # Toomas mentioned possibly publishing a map topic. Not sure if this is the 
        # way to go for initial planning
        Temp = Range
        self.map_sub = rospy.Subscriber('/map_topic', Temp, self.map_callback)
        #actuation
        self.fly_cmd = Twist()
        self.z_thresh = 0

        #rotation
        self.prev_angle = 0

        #SET THRESHOLD
        self.threshold = 1
        self.initialize_hover = True

    def load_environmental_map(self, path):
        """
        Task-Planning will provide a map with height data for us to generate a path plan. This map will be
        a descritization of the environment. There is a height limit we cannot surpass, therefore locations
        with altitudes that exceed this limit will be considered "occupied"

        Parameters:
        path (string): path to location of saved map. Pressumably somewhere in Task-Planning

        Returns:
        None
        """
        pass

    def odom_callback(self, msg: any) -> None:
        """
        this callback function will take in the odometry to determine where we are in space. We will use
        these to set the actuations in a PD controller for x, y, and z position

        # Considerations: We now have height data. Do we control of the given height data, the laser sensor, 
        combination of both? Theoretically we can penalize both, perhaps weight sensor data much more?
        
        Parameters:
        msg (Odometry): odometry object containing the current pose of the drone

        Returns: 
        None
        """        
        # determine where the drone is currently located
        if self.debug:
            location = self.debug_location
        else:
            location = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        if self.initialize_hover:
            # bring the drone up to the flying plane before planning
            self.fly_cmd.linear.x=0
            self.fly_cmd.linear.y=0
            self.fly_cmd.linear.z=1
            self.vel_pub.publish(self.fly_cmd)
            
            if location[2] >= self.flight_height:
                self.initialize_hover = False
            return

        # only execute this logic if we haven't hit all waypoints yet
        if not self.done_travelling and self.path_found:
            # if we've gotten close enough to the target waypoint, plan for next waypoint
            if self.debug:
                print(f"absolute difference: {np.absolute(self.waypoints[self.waypoint_index] - location)}")

            # if np.all(np.absolute(self.waypoints[self.waypoint_index] - location) < np.array([0.2, 0.2, 0.2])):
            if self.next_waypoint:
                if self.debug:
                    print("========================================================")
                    print(f"Drone made it to new waypoint location: {self.waypoints[self.waypoint_index]}")
                self.waypoint_index += 1
                self.path_plan = True
                self.next_waypoint = False
                # condition to check if we've hit all waypoints
                if self.waypoint_index == len(self.waypoints):
                    self.done_travelling = True
                    return

            # if we still haven't hit all waypoints, continue logic execution
            if self.path_plan:
                next_waypoint = self.waypoints[self.waypoint_index]

                # stop the drone while we path plan
                print(f"Drone is planning new route to: {next_waypoint}")
                self.fly_cmd.linear.x=0
                self.fly_cmd.linear.y=0
                self.fly_cmd.linear.z=0
                self.vel_pub.publish(self.fly_cmd)
                
                # determine a path to the next location, sets class variable self.path
                self.a_star_planner(next_waypoint, location)

                # A star failed to find a path to the next waypoint. Stay in place
                if self.path is None:
                    print("Failed to find path. Staying in place")
                    self.path_found = False
                    # Let the drone just stay in place
                    self.fly_cmd.linear.x=0
                    self.fly_cmd.linear.y=0
                    self.fly_cmd.linear.z=0
                    self.vel_pub.publish(self.fly_cmd)
                    return
                
                self.path_plan = False
                print(f"Drone found the following route: {self.path}")
                self.vis_paths(self.path)
                self.path_index = 0   

            # if we are at the next point in the path, aim for the subsequent point
            if np.all(np.absolute(self.path[self.path_index] - location) < np.array([0.2, 0.2, 0.2])):
                self.path_index += 1
                if self.path_index == len(self.path):
                    self.next_waypoint = True
                    return
            path_target = self.path[self.path_index] #next node (obj) with location np 
            # print(f"path target: {path_target}")

            # fake drone movement if debugging
            if self.debug:
                print("Drone is moving....")
                self.debug_location = path_target + 0.02*np.random.rand(*path_target.shape)
                print(f"Drone now at location: {self.debug_location}")
            else:
                # pass in velocity commands to move drone, implement x, y value PD
                errors = path_target - location             
                self.fly_cmd.linear.x , self.fly_cmd.linear.y = self.kp*errors[:-1] - self.kd*self.prev_v

                #if env is too close to drone
                if self.within_threshold:
                    print('violated threshold')
                    self.fly_cmd.linear.z = self.threshold-self.ground_dist
                # nominal threshold present, control global z value
                else:
                    z_error = path_target[2] - location[2]
                    self.fly_cmd.linear.z = self.kp * z_error 
                
                # set previous velocity for D term in PD
                self.prev_v = np.array(self.fly_cmd.linear.x, self.fly_cmd.linear.y)
                self.vel_pub.publish(self.fly_cmd)

        # all waypoints reached, no need to do anything anymore
        else:
            if not self.path_found:
                print(f"We failed to find paths. We stopped at waypoint {self.waypoints[self.waypoint_index]}")
            else:
                print("We've hit all waypoints! Should I return to base?")

    def face_forward_control(self, velocity_vector, pose):
        '''
        Controller to make the drone face in the direction of the next waypoint

        inp: velocity_vector, np.array[x_component,y_component]
        out: how much the drone should rotate
        '''

        #get current angle of the drone
        front_angle = pose.pose.orientation.z
        #get angle of velocity vector to vector the angles are defined by 
        a = np.array([1,0])
        velocity_angle = np.arccos(np.dot(a,velocity_vector)/(np.linalg.norm(a)*np.linalg.norm(velocity_vector)))
        #angle to rotate
        rotation_angle = velocity_angle - front_angle
        #Control
        self.fly_cmd.angular.z = self.kp*rotation_angle - self.kd*self.prev_angle
        #prior angular velocity?
        self.prev_angle = 0
        #publish angle command
        self.vel_pub.publish(self.fly_cmd)


    def range_callback(self, msg) -> None:
        """
        View above docstring. TBD how we interface these two. Perhaps we shoot for the target z provided
        in the waypoint, unless we see we are too close to the ground so we then switch the target_z to be 
        controlled by the laser. Perhaps 1m buffer. I.e., aim to fly 2m above the ground, if we are less than
        1m above the ground, then we let laser control take over. Boolean state?

        Parameters:
        msg (Range): Range object containing distance to ground data

        Returns:
        None
        """
        self.ground_dist = msg.range
        
        #if height is below threshold, increase the height of the 
        self.within_threshold = self.ground_dist < self.threshold
            
    def actuation(self, new_fly_cmd):
        '''
        Sets the actuation for the drone.
        Note: make sure threshold is being actuated before odometry -- when moving at high speeds more important to make sure we're being safe before going in right direction

        Parameters:
        new_fly_cmd: Twist()
        '''
        #publish fly command
        self.vel_pub.publish(new_fly_cmd)
        #set self.fly_cmd to the current fly cmds
        self.fly_cmd = self.new_fly_cmd

    def map_callback(self, msg):
        """
        Extract map information from the map topic. Unsure if this is the best way to approach this. 
        May want a static map for initial planning. Perhaps a different script handles the dynamic
        updates
        """
        pass

    def meters_to_grid(self, x: float, y: float) -> tuple:
        """
        Takes in a location in meters and outputs the discretized grid location.

        Parameters:
        x, y (float): a x, y location in the continuous state space

        Returns:
        row, col (int): a coordiante in the discretized state space
        """
        row = (x - self.min_x)/(self.max_x - self.min_x)*self.max_row
        col = (y - self.min_y)/(self.max_y - self.min_y)*(self.max_col)
        return int(row), int(col)

    def grid_to_meters(self, row: int, col: int) -> tuple:
        """
        Takes in a grid coordinate and returns a location in meters.

        Parameters:
        row, col (int): a row, col location in the discretized state space

        Returns:
        x, y (float): a x, y location in the continuous state space
        """
        x = (self.max_x - self.min_x)*row/self.max_row + self.min_x
        y = (self.max_y - self.min_y)*col/self.max_col + self.min_y
        return x, y

    def extract_path(self, final_node: Node) -> None:
        """
        Takes in the goal node and iteratively backtracks until we reach the start node
        Sets the extracted path as the class variable self.path

        Parameters:
        final_node (Node): the goal node

        Returns:
        None
        """
        node = final_node
        points = []
        # backtrack through all nodes until we get to the orign node
        while True:
            row, col = node.location[0], node.location[1]
            # extract the height from the height map, +2 for buffer
            x, y = self.grid_to_meters(row, col)
            points.append(np.array([x, y, self.flight_height]))
            node = node.parent
            if node is None:
                break
        points.reverse()
        self.path = points

    def get_neighbors(self, loc: np.ndarray) -> set:
        """
        Checks the neighbors of the current grid location we are at and returns the set that are free.

        Parameters:
        loc (np.ndarray): row, col location in the 2D discritized representation
        visited (set): tuples of row, col locations that have already been visited

        Returns:
        free (set): set containing tuples of free neighbors.
        """
        free = set()
        # eight neighboring grid locations
        translations = (np.array([-1, -1]), np.array([-1, 0]), np.array([-1, 1]), np.array([0, 1]), 
                np.array([0, -1]), np.array([1, -1]), np.array([1, 0]), np.array([1, 1]))
        for translation in translations:
            new_loc = loc + translation
            # check validity of new location (make sure we didn't go outside the map)
            if np.any(new_loc < 0) or np.any(new_loc == np.array(self.occupancy_grid.shape)):
                continue
            # only add unoccupied locations and nodes unvisited by the A-star algorithm
            if self.occupancy_grid[new_loc[0], new_loc[1]] == 0:
                free.add(tuple(new_loc))
        return free

    def a_star_planner(self, goal: np.ndarray, start=np.array([0, 0]), debug=False) -> None:
        """
        This function will generate a path from location start to location goal. It will generate a plan
        that avoids obstacles by planning over the provided map in self.map

        self.map is a 2D matrix providing height data, some of which will be above a threshold to create an
        occupancy grid. 

        Parameters: 
        start (np.ndarray): x, y location the drone is starting from.
        goal (np.ndarray): x, y location the drone is to arrive

        Returns:
        path (list): Nodes containing the (x, y, z) coordinates for the drone to follow along its path
        """
        self.path_found = False
        # testing will pass in row, col grid locations directly
        if debug:
            start_grid = start
            end_grid = goal
        else:
            start_grid = np.array(self.meters_to_grid(*start[:2]))
            end_grid = np.array(self.meters_to_grid(*goal[:2]))

        # note that that both nodes and the priority queue operate in u,v space, not x,y space
        start_node = Node(start_grid)
        # list of partial paths in format of [([partial_path_1], cost_so_far_1, cost_to_go_1), ... ]
        priority_queue = [([start_node], 0.0, start_node.get_dist(end_grid))]
        # set containing all the UV spaces we've visited so far
        visited = set()

        # continue iteration process until we find the path or we exhaust the entirety of the search space
        while priority_queue and not self.path_found:
            #priority queue sorted from worst to best, so best will be at end
            best_partial_path = priority_queue.pop()
            # final node in the best partial path
            best_path_tail = best_partial_path[0][-1]
            
            # found goal, extract the path
            if (best_path_tail.location == end_grid).all():
                self.extract_path(best_path_tail)
                self.path_found = True
                break
                
            # check if last node in partial path has already been visited
            if tuple(best_path_tail.location) in visited:
                continue

            # if it hasn't been visited, get it's neighbors and expand paths
            visited.add(tuple(best_path_tail.location))
            neighbors = self.get_neighbors(best_path_tail.location)
            # only take the neighbors that haven't been visited yet
            new_neighbors = neighbors.difference(visited)
            for neighbor in new_neighbors:
                # create node instance for each new neighbor to create new partial path
                new_node = Node(np.array(neighbor), parent=best_path_tail)
                cost_to_go = new_node.get_dist(end_grid)
                cost_so_far = best_partial_path[1] + new_node.get_dist(new_node.parent.location)
                partial_path = best_partial_path[0] + [new_node]
                priority_queue.append((partial_path, cost_so_far, cost_to_go))
                
            # sort priority queue based on cost_so_far + cost_to_go, reverse to make popping more efficient
            priority_queue.sort(reverse=True, key=lambda x: x[1] + x[2])
        
        # no path found
        if not self.path_found:
            self.path = None

    def rrt_planner(self, goal: np.ndarray, start=np.array([0, 0])) -> None:
        """
        This function will generate a path from location start to location goal by using RRT. It will 
        generate a plan that avoids obstacles by planning over the provided map in self.map. Stores the
        path in self.path.

        Parameters: 
        start (np.ndarray): x, y location the drone is starting from.
        goal (np.ndarray): x, y location the drone is to arrive

        Returns:
        None
        """
        start = start[:2]
        goal =goal[:2]
        path_found = False
        valid_set = {Node(start)}
        free_grid_points = np.argwhere(self.occupancy_grid == 0)
        d = 5
        tol = d*1.0

        # Repeat RRT graph generation until goal reached
        while not path_found:
            # genereate random point in space, goal biased-parameter
            if np.random.uniform() < 0.0:
                pos_rand = goal
                goal_sampled = True
            else:
                goal_sampled = False
                rand_int = np.random.randint(free_grid_points.shape[0])
                row, col = free_grid_points[rand_int:rand_int+1].flatten()
                # x, y coordinate of the grid location
                pos_rand = np.array(self.grid_to_meters(row, col))

            # find the closet node in valid_set to this random point
            closest = min(valid_set, key=lambda x: x.get_dist(pos_rand))
            old_pos = closest.location

            if goal_sampled and closest.get_dist(goal) < d:
                # super close to the goal, allow use to select the goal as the new position
                new_pos = goal
            else:
                # get unit vector from closest to random point
                unit_vec = closest.get_vec(pos_rand)
                # step in vector direction by amount d
                new_pos = closest.location + unit_vec*d

            # if collides, ignore, else add to valid set and create parent child dependency
            if self.collision_detector(old_pos, new_pos, d):
                continue
            new_node = Node(new_pos, parent=closest)
            valid_set.add(new_node)

            # if in goal region, path_found = True, else, repeat
            if np.linalg.norm(new_pos - goal) < tol:
                path_found = True
                print('path found')
                self.extract_path_RRT(new_node)

    def extract_path_RRT(self, final_node):
        node = final_node
        points = []
        while True:
            points.append((node.location[0], node.location[1], self.flight_height))
            node = node.parent
            if node is None:
                break
        points.reverse()
        self.path = points

    def collision_detector(self, curr: np.ndarray, dest: np.ndarray, d: float) -> bool:
        '''
        Given a start position and a destination returns if the current path subsection is in collision

        Parameters: 
        curr (np.ndarray): The start of the path we are checking collisions for
        dest (np.npdarray): The location we'd like to connect to the tree 
        d (float): the step size used for RRT

        Returns: 
        collision (bool): if path is in collision with obstacle
        '''
        # get direcitonal normlized vector
        sd_unit_vec = (dest-curr)/np.linalg.norm(dest-curr)
        collision = False
        pt = curr

        # iterate through all possible points spaced a distance step apart
        step = d/10. # WILL NEED TUNING

        # while current pt is not within a viable range of the destination
        while not collision and np.linalg.norm(dest-pt) > step:
            # update the point location, create points along line from curr to dest
            pt = pt + step*sd_unit_vec
            # check if point is in collision
            pxls = self.meters_to_grid(*pt)
            collision = not self.occupancy_grid[pxls[0], pxls[1]] == 0
        return collision

    def vis_paths(self, path):
        path_grid = np.array([self.meters_to_grid(point[0], point[1]) for point in path])
        rows, cols = path_grid[:, 0], path_grid[:, 1]
        new_grid = self.occupancy_grid
        new_grid[rows, cols] = 100
        cv2.imshow('new grid', new_grid)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# For testing purposes
def grid_to_meters(max_x: int, min_x: int, max_y: int, min_y: int, row: int, col: int) -> tuple:
    """
    Takes in a grid coordinate and returns a location in meters.

    Parameters:
    row, col (int): a row, col location in the discretized state space

    Returns:
    x, y (float): a x, y location in the continuous state space
    """
    x = (max_x - min_x)*row/max_row + min_x
    y = (max_y - min_y)*col/max_col + min_y
    return x, y

if __name__ == "__main__":
    path = '../occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    cv2.imshow("original occupancy", occupancy_image)
    reduced_occupancy = skimage.measure.block_reduce(occupancy_image, (5, 5), np.max)
    cv2.imshow('reduced occupancy', reduced_occupancy)
    dilated_ococupancy = cv2.dilate(reduced_occupancy, np.ones((7, 7), np.uint8))
    cv2.imshow('dilated occupancy', dilated_ococupancy)

    xmin, xmax = -100, 100
    ymin, ymax = -100, 100
    max_row, max_col = np.array(dilated_ococupancy.shape) - 1

    # generate some waypoints to follow
    free_grid = np.transpose(np.where(dilated_ococupancy == 0))
    rows = np.random.choice(free_grid.shape[0], 5, replace=False)
    waypoints_grid = free_grid[rows, :]

    # convert the waypoints into cartesian space
    waypoints = []
    for waypoint in waypoints_grid:
        x, y = grid_to_meters(xmax, xmin, ymax, ymin, waypoint[0], waypoint[1])
        waypoints.append((x, y, 10))

    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('AStar_planner', anonymous=True)
        NAV = Navigator(dilated_ococupancy, xmin, xmax, ymin, ymax, waypoints)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
