import rospy
import numpy as np
import cv2
import skimage.measure
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32



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
        self.pub = rospy.Publisher('/waypoint_topic', Point32, queue_size=1)
        self.sub = rospy.Subscriber('/ground_truth/state', Odometry, self.callback)
        self.sub_nav = rospy.Subscriber('/done_travelling', Bool, self.done_callback)
        self.pub_next = True
        self.flight_height = 10

    def callback(self, msg):
        if self.waypoint_index == len(self.waypoints):
            print("Done Travelling. Should I return to Base?")
            return 
        
        if self.pub_next:
            new_waypoint = self.waypoints[self.waypoint_index]
            point = Point32()
            point.x = new_waypoint[0]
            point.y = new_waypoint[1]
            point.z = self.flight_height
            print(f"Publishing point: {point.x, point.y, point.z}")
            self.pub.publish(point)
            self.waypoint_index += 1
            self.pub_next = False
        self.rate.sleep()

    def done_callback(self, msg):
        if msg.data:
            self.pub_next = True


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
    
    try:
        # create the navigator object, pass in important mapping information
        rospy.init_node('fake_pub', anonymous=True)
        rate = rospy.Rate(10)
        PUB = FakeNavPub(waypoints=waypoints, rate=rate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass