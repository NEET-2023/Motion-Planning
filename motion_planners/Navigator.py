import rospy
import cv2
import numpy as np
import skimage.measure
import sys
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool
from planners import Planners
sys.path.insert(0, '/home/frankgon/catkin_ws/src/Motion-Planning/')
from low_level_controller.PD import PD
from low_level_controller.Pure_Pursuit import PurePursuit
from low_level_controller.Face_Forward import FaceForward

class Navigator():
    def __init__(self, algo: str, occupancy_grid: np.ndarray, world_dims: tuple, rate: rospy.Rate, waypoint = None, debug=False):
        # Map inforation intializations
        self.occupancy_grid = occupancy_grid
        self.max_row, self.max_col = np.array(occupancy_grid.shape) - 1
        self.ground_dist = 0
        self.flight_height = 10
        self.world_dims = world_dims
        self.max_z = 100 
        # Tracking of which waypoint/point in path we are following
        self.waypoint = waypoint 
        self.done_travelling = Bool()
        self.done_travelling.data = False
        self.path_plan = True
        self.path_found = True
        self.path = None 
        self.path_index = 0
        self.last_waypoint = None
        # Planner
        self.planner = Planners(algo, world_dims, occupancy_grid, self.flight_height)
        # Low Level Controllers
        # self.llc = PD()
        self.llc = PurePursuit()
        self.ffc = FaceForward()
        # PD controller initializations
        self.kp = 1.0
        self.kd = 0
        self.max_v = 1
        self.prev_v_world = np.array([0, 0, 0])
        self.below_height_threshold = False
        # PD control for z independently
        self.max_vz = 1
        # Face Forward control variables
        self.prev_angular_z = 0
        self.facing_forward = False
        # ROS relevant initializations
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
        self.waypoint_sub = rospy.Subscriber('/waypoint_topic', Point, self.waypoint_callback)
        self.finished_pub = rospy.Publisher('/done_travelling', Bool, queue_size=1)
        self.map_sub = rospy.Subscriber('/map_topic', OccupancyGrid, self.occupancy_callback)
        # fly command initialization
        self.fly_cmd = Twist()
        # thresholds
        self.threshold = 1
        self.initialize_hover = True

        # debugging publishers
        self.traj_pub = rospy.Publisher('/trajectory', Marker, queue_size=1)

        # rate parameters
        self.rate = rate

    def waypoint_callback(self, msg: Point) -> None:
        """
        Task-Planning will provide the waypoints we are to follow. This function will override the waypoint 
        we are interested in planning and set the necessary flags in order to proceed properly.
        """
        self.waypoint = np.array([msg.x, msg.y, self.flight_height])
        if not np.all(self.last_waypoint == self.waypoint):
            self.done_travelling.data = False
            self.path_plan = True
            self.path_found = True
            self.last_waypoint = self.waypoint

    def occupancy_callback(self, msg):
        """
        Create a 2D occpancy grid from the provided 1D list of blocking probabilities. Assumes a value of 
        0 means the space is open.

        Parameters:
        msg (OccupancyGrid): has one attribute called data which contains the blocking probabilities

        Returns:
        None
        """
        occupancies = msg.data
        size = len(occupancies)
        self.occupancy_grid = np.array(occupancies).reshape((size**0.5, size**0.5))

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

    def odom_callback(self, msg: Odometry) -> None:
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
        # no waypoints provided yet
        if self.waypoint is None:
            print("No waypoint provided")
            return
        # no occupancy grid provided yet
        if self.occupancy_grid is None:
            print("No occupancy grid provided")

        pose = msg.pose.pose
        # determine where the drone is currently located
        location = np.array([pose.position.x, pose.position.y, pose.position.z])

        if self.initialize_hover:
            # bring the drone up to the flying plane before planning
            self.fly_cmd.linear.x = 0
            self.fly_cmd.linear.y = 0
            self.fly_cmd.linear.z = 1 if location[2] < self.flight_height else -1
            self.vel_pub.publish(self.fly_cmd)
            
            # condition to terminate hover sequence and let the rest of motion planning take over
            if abs(location[2] - self.flight_height) < 0.1:
                self.initialize_hover = False
            return

        # only execute this logic if we haven't hit all waypoints yet
        if not self.done_travelling.data and self.path_found:
            # if we need to plan a path, go ahead and do that
            if self.path_plan:
                # stop the drone while we path plan
                print(f"Drone is planning new route to: {self.waypoint}")
                self.fly_cmd.linear.x=0
                self.fly_cmd.linear.y=0
                self.fly_cmd.linear.z=0
                self.fly_cmd.angular.z = 0
                self.vel_pub.publish(self.fly_cmd)
                
                # determine a path to the next location, sets class variable self.path
                self.path_found, self.path = self.planner.plan(self.waypoint, location)

                # planner failed to find a path to the next waypoint. Stay in place
                if self.path is None:
                    print("Failed to find path. Staying in place")
                    self.path_found = False
                    # Let the drone just stay in place
                    self.fly_cmd.linear.x=0
                    self.fly_cmd.linear.y=0
                    self.fly_cmd.linear.z=0
                    self.fly_cmd.angular.z = 0
                    self.vel_pub.publish(self.fly_cmd)
                    return
                
                # pass the path information into the low level controller
                self.llc.path = self.path
                if self.llc.type == "PD": 
                    self.llc.path_index = 0
                # CURRENT BUGS WITH PP
                # 1. PP STOPS DISTANCE LOOK_AHEAD FROM WAYPOINT
                # 2. SOMETIMES DRONE FREAKS OUT AND I HAVE NO IDEA WHY
                elif self.llc.type == "PP": 
                    self.llc.stop = False
                    self.llc.path_segments = np.array([[self.path[i][0], 
                                                        self.path[i][1], 
                                                        self.path[i+1][0], 
                                                        self.path[i+1][1]] 
                                                        for i in range(len(self.path)-1)])
                self.path_plan = False

                # some RVIZ visualizations for our sake
                print(f"Drone found the following route: {self.path}")
                self.vis_paths_image(self.path)
                self.publish_path_rviz(self.path)
                self.path_index = 0   

            # Passes relevant information into a low-level controller to move the drone along the path
            self.llc.pose = pose
            self.llc.ground_dist = self.ground_dist
            self.llc.within_threshold = self.within_threshold
            self.llc.threshold = self.threshold
            self.llc.prev_v_world = self.prev_v_world
            self.done_travelling.data, v_world = self.llc.actuate()
            self.prev_v_world = v_world

            # second low-level controller so the drone faces the direction of motion
            self.ffc.prev_angular_z = self.prev_angular_z
            v_drone, omega_z = self.ffc.face_forward_control(v_world, pose)
            #######################################################
            # potentailly face motion direction first and then move
            #######################################################

            self.fly_cmd.linear.x, self.fly_cmd.linear.y, self.fly_cmd.linear.z = v_drone
            self.fly_cmd.angular.z = omega_z
            self.prev_angular_z = omega_z
            self.vel_pub.publish(self.fly_cmd)

        # all waypoints reached, no need to do anything anymore
        else:
            if not self.path_found:
                print(f"We failed to find paths. We stopped at waypoint: {self.waypoint}")
                return False
            else:
                print("Reached Waypoint!")
                self.done_travelling.data = True
                self.finished_pub.publish(self.done_travelling.data)
                rate.sleep()
                return True

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

    def vis_paths_image(self, path: list) -> None:
        """
        Takes in the path generated by Planner and overlays it on an image of the occupancy grid

        Pamameters:
        path (list): path of points the drone is to follow

        Returns:
        None
        """
        path_grid = np.array([meters_to_grid(self.world_dims, self.max_row, self.max_col, point[0], point[1]) for point in path])
        rows, cols = path_grid[:, 0], path_grid[:, 1]
        new_grid = self.occupancy_grid
        new_grid[rows, cols] = 100
        cv2.imshow('new grid', new_grid)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def publish_path_rviz(self, path: list) -> None:
        """
        Takes the generated path and publishes a Rviz visualization

        Pamameters:
        path (list): path of points the drone is to follow

        Returns:
        None
        """
        print("Publishing trajectory")
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"
        marker = Marker()
        marker.header = header
        marker.ns = "trajectory"
        marker.id = 2
        marker.type = marker.LINE_STRIP
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        for p in path:
            pt = Point()
            pt.x = p[0]
            pt.y = p[1]
            pt.z = p[2]
            marker.points.append(pt)
        self.traj_pub.publish(marker)

# Functions for testing purposes
def grid_to_meters(world_dims: tuple, max_row: int, max_col: int, row: int, col: int) -> tuple:
    """
    Takes in a grid coordinate and returns a location in meters.

    Parameters:
    row, col (int): a row, col location in the discretized state space

    Returns:
    x, y (float): a x, y location in the continuous state space
    """
    min_x, max_x, min_y, max_y = world_dims
    x = (max_x - min_x)*row/max_row + min_x
    y = (max_y - min_y)*col/max_col + min_y
    return x, y

def meters_to_grid(world_dims: tuple, max_row: int, max_col: int, x: float, y: float) -> tuple:
    """
    Takes in a location in meters and outputs the discretized grid location.

    Parameters:
    x, y (float): a x, y location in the continuous state space

    Returns:
    row, col (int): a coordiante in the discretized state space
    """
    min_x, max_x, min_y, max_y = world_dims
    row = (x - min_x)/(max_x - min_x)*max_row
    col = (y - min_y)/(max_y - min_y)*(max_col)
    return int(row), int(col)

if __name__ == "__main__":
    path = '../occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    # cv2.imshow("original occupancy", occupancy_image)
    reduced_occupancy = skimage.measure.block_reduce(occupancy_image, (5, 5), np.max)
    # cv2.imshow('reduced occupancy', reduced_occupancy)
    dilated_ococupancy = cv2.dilate(reduced_occupancy, np.ones((7, 7), np.uint8))
    # cv2.imshow('dilated occupancy', dilated_ococupancy)

    # xmin, xmax, ymin, ymax
    world_dims = (-100, 100, -100, 100)
    max_row, max_col = np.array(dilated_ococupancy.shape) - 1

    # generate some waypoints to follow
    free_grid = np.transpose(np.where(dilated_ococupancy == 0))
    rows = np.random.choice(free_grid.shape[0], 1, replace=False)
    waypoints_grid = free_grid[rows, :]

    # convert the waypoints into cartesian space
    waypoints = []
    for waypoint in waypoints_grid:
        x, y = grid_to_meters(world_dims, max_row, max_col, waypoint[0], waypoint[1])
        waypoints.append((x, y, 10))

    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('Planner', anonymous=True)
        rate = rospy.Rate(0.5)
        NAV = Navigator('a_star', dilated_ococupancy, world_dims, rate, waypoint=None)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
