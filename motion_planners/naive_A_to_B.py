import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class NaiveAtoB():
    """
    Uses a PD controller to navigate between 2D locations while controlling the z-value to self.z_target
    """
    def __init__(self):
        # high-level controller logic initializations
        self.waypoints = self.generate_waypoints()
        self.threshold = 0.1
        self.waypoint_index = 0
        self.target_z = 1
        # PD controller initializations
        self.kp = 0.8
        self.kd = 0
        self.max_v = 2
        self.prev_v = np.array([0, 0])
        self.prevz = 0
        # ROS relevant initializations
        self.pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.init_node('naiveAtoB', anonymous=True)
        rospy.Subscriber('/ground_truth/state', Odometry, self.callback)

    def callback(self, msg):
        """
        Calculates the command velocities as a function of 2D location error and height error
        Publishes a Twist data type to set the velocity command

        Inputs: msg (Odometry)
        Returns: None
        """
        # uses ground truth data to get the position
        # TODO: use IMU data + starting position to track location
        loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        # determine if we are at the target waypoint to update the pointer
        if self.at_goal(loc):
            self.waypoint_index += 1
            self.waypoitn_index % len(self.waypoints)
        waypoint = self.waypoints[self.waypoint_index]

        # PD Controller
        error_2D = waypoint - loc
        v = self.kp*error_2D - self.kd*self.prev_v
        self.prev_v = v

        error_Z = self.target_z - msg.pose.pose.position.z
        vz = self.kp*error_Z - self.kd*self.prev_vz
        self.prev_vz = vz

        # Set Twist values
        cmd_vel = Twist()
        cmd_vel.linear.x = min(v[0], self.max_v)
        cmd_vel.linear.y = min(v[1], self.max_v)
        cmd_vel.linear.z = min(vz, self.max_v)

    def at_goal(self, loc):
        """
        Checks for a match between the x and y coordinate of the location and the waypoint
        
        Inputs: loc (np.array 1x2)
        Returns: Boolean
        """
        current_waypoint = np.array(self.waypoints[self.waypoint_index])
        return np.linalg.norm(loc - current_waypoint) < self.threshold

    def generate_waypoints(self):
        """
        Creates a set of 2D coordinates for the drone to follow. For now hardcoded
        Returns: Nx2 numpy array
        """
        return np.array([0, 0], [5, 5], [-5, 5], [10, 10])
