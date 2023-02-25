import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range


class Navigator():
    def __init__(self):
        # Map inforation intializations
        self.waypoints = None
        self.height_map = None
        self.occupancy_grid = None
        self.min_x, self.max_x = 0, 0
        self.min_y, self.max_y = 0, 0
        self.max_z = None
        # PD controller initializations
        self.kp = 1.0
        self.kd = 0
        self.max_v = 1
        self.prev_v = np.array([0, 0])
        # May or may not control z independently. TBD
        self.max_vz = 1
        self.prev_vz = 0
        # ROS relevant initializations
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
        # Toomas mentioned possibly publishing a map topic. Not sure if this is the 
        # way to go for initial planning
        Temp = None
        self.map_sub = rospy.Subscriber('/map_topic', Temp, self.map_callback)


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

    def odom_callback(self, msg):
        """
        this callback function will take in the odometry to determine where we are in space. We will use
        these to set the actuations in a PD controller for x, y, and z position

        # Considerations: We now have height data. Do we control of the given height data, the laser sensor, 
        combination of both? Theoretically we can penalize both, perhaps weight sensor data much more?
        """
        pass

    def range_callback(self, msg):
        """
        View above docstring. TBD how we interface these two. Perhaps we shoot for the target z provided
        in the waypoint, unless we see we are too close to the ground so we then switch the target_z to be 
        controlled by the laser. Perhaps 1m buffer. I.e., aim to fly 2m above the ground, if we are less than
        1m above the ground, then we let laser control take over. Boolean state?
        """
        pass

    def map_callback(self, msg):
        """
        Extract map information from the map topic. Unsure if this is the best way to approach this. 
        May want a static map for initial planning. Perhaps a different script handles the dynamic
        updates
        """
        pass

    def a_star_planner(self, start, goal):
        """
        This function will generate a path from location start to location goal. It will generate a plan
        that avoids obstacles by planning over the provided map in self.map

        self.map is a 2D matrix providing height data, some of which will be above a threshold to create an
        occupancy grid. 
        """
        pass

if __name__ == "__main__":
    try:
        NAV = Navigator()
        map_path = "../../Task-Planning/something"
        NAV.load_environmental_map(map_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
