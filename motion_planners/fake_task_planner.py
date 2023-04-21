import rospy
import numpy as np
import cv2
import skimage.measure
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point



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

class FakeNavPub():
    def __init__(self, waypoints, rate):
        self.waypoints = waypoints
        self.waypoint_index = 0
        self.rate = rate
        # ROS publishers to execute other nodes
        self.waypoint_pub = rospy.Publisher('/waypoint_topic', Point, queue_size=1)
        self.place_sensor_pub = rospy.Publisher('/place_sensor', Bool, queue_size=1)
        self.pickup_sensor_pub = rospy.Publisher('/pickup_sensor', Bool, queue_size=1)
        # ROS subscribers to run this script
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.decision_callback)
        self.done_travelling_sub = rospy.Subscriber('done_travelling', Bool, self.done_travelling_callback)
        self.sensor_placed_sub = rospy.Subscriber('/sensor_placed', Bool, self.sensor_placed_callback)
        self.sensor_pickedup_sub = rospy.Subscriber('/sensor_pickedup', Bool, self.sensor_pickedup_callback)
        # Boolean and state flags for the state machine
        self.done_travelling = False
        self.sensor_placed = False
        self.sensor_pickedup = False
        self.flight_height = 10
        self.state = "Initialized"
        self.mode = "Placing"

    def decision_callback(self, msg):
        """
        Callback function responsible for high level decision-making for the drone. It reads in odometry as a 
        dummy variable and makes decisions based on defined state flags. 

        Paramters:
        msg (Odometry): state information about the drone

        Returns:
        None
        """
        # state upon first initialization of the task planner
        if self.state == "Initialized":
            print("Initializing Everything")
            self.state = "Travelling"

        # state when we are travelling between waypoints
        if self.state == "Travelling":
            # hit all waypoints, finish execution
            if self.waypoint_index == len(self.waypoints):
                self.state = "Done"
                return 
            # not at last waypoint yet, publish the current waypoint of interest
            new_waypoint = self.waypoints[self.waypoint_index]
            point = Point()
            point.x = new_waypoint[0]
            point.y = new_waypoint[1]
            point.z = self.flight_height
            print(f"Publishing point: {point.x, point.y, point.z}")
            self.waypoint_pub.publish(point)
            # give the navigator file time to register the new waypoint
            self.rate.sleep()

            # reached the waypoint, place or retrieve the sensor
            if self.done_travelling:
                self.state = "Sensor"
                return
        
        # state when we are to interact with the sensor in some way
        if self.state == "Sensor":
            msg = Bool()
            msg.data = True
            # broadcast if we want to place the sensor
            if self.mode == "Placing":
                self.place_sensor_pub.publish(msg)
            # broadcast if we want to pickup the sensor
            if self.mode == "Pickup":
                self.pickup_sensor_pub.publish(msg)
            
            # if we have successfully maniuplated the sensor, move on
            if self.sensor_placed or self.sensor_pickedup:
                self.state == "Travelling"
                self.waypoint_index += 1

                # reset all state flags
                self.done_travelling = False
                self.sensor_placed = False
                self.sensor_pickedup = False
                return
            
        if self.state == "Done":
            print("Drone has finished operations")
            return

    def done_travelling_callback(self, msg):
        if msg.data:
            self.done_travelling = True

    def sensor_placed_callback(self, msg):
        if msg.data:
            self.sensor_placed = True

    def sensor_pickedup_callback(self, msg):
        if msg.data:
            self.sensor_pickedup = True

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
    rows = np.random.choice(free_grid.shape[0], 5, replace=False)
    waypoints_grid = free_grid[rows, :]

    # convert the waypoints into cartesian space
    waypoints = []
    for waypoint in waypoints_grid:
        x, y = grid_to_meters(world_dims, max_row, max_col, waypoint[0], waypoint[1])
        waypoints.append((x, y, 10))

    test_waypoints = [[0, -20, 10], [20, -20, 10], [-20, -20, 10]]
    
    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('fake_pub', anonymous=True)
        rate = rospy.Rate(1)
        PUB = FakeNavPub(waypoints=waypoints, rate=rate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass