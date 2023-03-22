import rospy
import numpy as np
from node import Node

class Planners():
    def __init__(self,  algo: str, world_dims: tuple, occupancy_grid: np.ndarray, flight_height: int, debug = False):
        self.algos = ['rrt','a_star']
        self.algo = algo
        self.min_x, self.max_x, self.min_y, self.max_y = world_dims
        self.occupancy_grid = occupancy_grid
        self.max_row, self.max_col = np.array(self.occupancy_grid.shape) - 1
        self.flight_height = flight_height
        self.debug = debug

    def plan(self, goal: np.ndarray, start: np.ndarray):
        path = None
        if self.algo not in self.algos:
            raise Exception('not a valid algo')
        
        if self.algo == 'rrt':
            status, path = self.rrt(goal,start,self.debug)
        
        elif self.algo == 'a_star':
            status, path = self.a_star(goal,start,self.debug)

        return status, path    

    def a_star(self, goal: np.ndarray, start: np.ndarray, debug=False) -> None:
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
        path_found = False
        path = None
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
        while priority_queue and not path_found:
            #priority queue sorted from worst to best, so best will be at end
            best_partial_path = priority_queue.pop()
            # final node in the best partial path
            best_path_tail = best_partial_path[0][-1]
            
            # found goal, extract the path
            if (best_path_tail.location == end_grid).all():
                path = self.extract_path(best_path_tail)
                path_found = True
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
        
        # return success status and path
        return path_found, path
        
    def extract_path(self, final_node: Node) -> None:
        """
        Takes in the goal node and iteratively backtracks until we reach the start node
        Sets the extracted path as the class variable self.path

        Parameters:
        final_node (Node): the goal node

        Returns:
        points (list): set of points between current location and the next waypoint
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
        return points
    
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
    
    def rrt_planner(self, goal: np.ndarray, start: np.ndarray) -> None:
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


if __name__ == '__main__':
    try:
        #planner = Planners(world_dims,algo,occupancy_grid)
        print('trying')
    except rospy.ROSInterruptException:
        pass