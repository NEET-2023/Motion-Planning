import numpy as np
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

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
        self.kp = 1.0
        self.kd = 0.6
        self.max_v = 2
        self.max_vz = 3
        self.prev_v = np.array([0, 0])
        self.prev_vz = 0
        # Relevant information for PD control from ground
        self.kp_z = 3.0
        self.kd_z = 0
        self.z = 0
        self.target_ground_dist = 1
        # ROS relevant initializations
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # FIGURE OUT HOW TO DEAL WITH THIS
        rospy.init_node('naiveAtoB', anonymous=True)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)

    def range_callback(self, msg):
        self.z = msg.range

    def odom_callback(self, msg):
        """
        Calculates the command velocities as a function of 2D location error and height error
        Publishes a Twist data type to set the velocity command

        Inputs: msg (Odometry)
        Returns: None
        """
        # TODO: use IMU data + starting position to track location
        # uses ground truth data to get the position
        loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        # determine if we are at the target waypoint to update the pointer
        if self.at_goal(loc):
            self.waypoint_index += 1
            self.waypoint_index %= len(self.waypoints)
        waypoint = self.waypoints[self.waypoint_index]

        # PD Controller
        error_2D = waypoint - loc
        v = self.kp*error_2D - self.kd*self.prev_v
        self.prev_v = v

        error_Z = self.target_z - self.z
        vz = self.kp_z*error_Z - self.kd_z*self.prev_vz
        self.prev_vz = vz

        # Set Twist values
        cmd_vel = Twist()
        cmd_vel.linear.x = max(min(v[0], self.max_v), -self.max_v)
        cmd_vel.linear.y = max(min(v[1], self.max_v), -self.max_v)
        cmd_vel.linear.z = max(min(vz, self.max_vz), -self.max_vz)

        # TODO: publish instead of print
        # print(cmd_vel)
        self.vel_pub.publish(cmd_vel)
        

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
        # successful waypoint navigation
        return np.array([[0, -45], [-10, -40], [10, -40], [10, -50], [-10, -50]])

if __name__ == '__main__':
    try:
        NAB = NaiveAtoB()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass