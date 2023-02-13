import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range

class ValleyAtoB():
    """
    Uses a PD controller to navigate between 2D locations while controlling the z-value to self.z_target
    """
    #Obstain Params
    SCAN_TOPIC = rospy.get_param('/scan')
    LASER_TOPIC = rospy.get_param('/sonar_height')

    def __init__(self):
        # high-level controller logic initializations
        self.waypoints = self.generate_waypoints()
        self.threshold = 0.1
        self.waypoint_index = 0
        self.target_z = 1
        # PD controller initialization
        self.kp = 0.8
        self.kd = 0
        self.max_v = 2
        self.prev_v = np.array([0, 0])
        self.prevz = 0
        # PD vertical controller initializations
        self.kp_z = 3.0
        self.kd_z = 0
        self.z =  0
        self.max_vz = 3
        #Lidar initializations
        self.angle_dict = {}
        # ROS relevant initializations
        self.fly_pub = rospy.Publisher('/cmd_vel', Twist)
        rospy.init_node('valleyAtoB', anonymous=True)

        #initialize pub/sub
        self.loc_subscriber = rospy.Subscriber('/ground_truth/state', Odometry, self.loc_callback)
        self.lidar_subscriber = rospy.Subscriber(ValleyAtoB.SCAN_TOPIC, LaserScan, self.lidar_callback)
        self.laser_subscriber = rospy.Subscriber(ValleyAtoB.LASER_TOPIC, Range, self.laser_callback)
    
    def loc_callback(self, msg):
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
            self.waypoint_index % len(self.waypoints)
        waypoint = self.waypoints[self.waypoint_index]

        # PD Controller
        # adjust target point
        # error_2D = waypoint - loc
        # v = self.kp*error_2D - self.kd*self.prev_v
        # self.prev_v = v

        # error_Z = self.target_z - self.z
        # vz = self.kp_z*error_Z - self.kd_z*self.prev_vz
        # self.prev_vz = vz

        # Set Twist values
        # cmd_vel = Twist()
        # cmd_vel.linear.x = max(min(v[0], self.max_v), -self.max_v)
        # cmd_vel.linear.y = max(min(v[1], self.max_v), -self.max_v)
        # cmd_vel.linear.z = max(min(vz, self.max_vz), -self.max_vz)

    def lidar_callback(self,Laser_data):
        '''
        Obtains the front facing lidar data
        Inp: Laser_data (LaserScan)
        Returns: None
        '''
        ranges = np.array(Laser_data.ranges)
        angle_min = Laser_data.angle_min
        angle_max = Laser_data.angle_max
        angle_difference = angle_max - angle_min
        angle_increment = Laser_data.angle_increment
        angle_list = np.arange(angle_min,angle_max,angle_increment)
        
        #obtain index of -pi/2 and pi/2
        left_upper_index = self.get_elem_index(-math.pi/2)
        left_lower_index = self.get_elem_index(-math.pi/2)
        right_upper_index = self.get_elem_index(math.pi/2)
        right_lower_index = self.get_elem_index(math.pi/2)

        #obtain scan values  
        left_range = ranges[left_lower_index:left_upper_index]
        left_scaling_array = angle_list[left_lower_index:left_upper_index]
        right_range = ranges[right_lower_index:right_upper_index]
        right_scaling_array= angle_list[right_lower_index:right_upper_index]

        left_x, left_y = self.get_clean_cartesian_data(left_range, left_scaling_array)
        right_x, right_y = self.get_clean_cartesian_data(right_range,right_scaling_array)
        
        #filter values 
        left_avg = np.mean(left_range)
        right_avg = np.mean(right_range)
    
    def get_clean_cartesian_data(self, sliced_data, scaling_array, threshold = None):
        '''
        Dropout datapoints if outliers (outside a certain threshold) if desired
        Inp: sliced_data ([distances]), scaling_array ([angles]), threshold (int)
        Returns: x ([int]), y ([int]) location of obstacles
        '''
        if threshold:
            indexes = np.where(sliced_data >= threshold)
            np.delete(sliced_data, indexes)
            np.delete(scaling_array, indexes)

        x = sliced_data*np.cos(scaling_array)
        y = sliced_data*np.sin(scaling_array)
        return x, y

        

    def get_elem_index(self,desired_angle,angle_min,increment):
        '''
        Obtains the index of the desired angle mapped to ranges
        inp: desired_angle (float)
        Returns: index (int)
        '''
        return int(((desired_angle - angle_min)/increment)+1)


    def laser_callback(self, laser_msg):
        '''
        Obtains downwards laser data, giving us information about the distance between the drone and the ground
        Inp: laser_msg (Range)
        Returns: None
        '''
        self.z = laser_msg.range
    
    def orient_to_goal(self,loc):
        '''
        Initially orients the drone towards the goal location as an initial step
        Inp: loc (no.array 1x2)
        Returns: None
        '''
        raise NotImplementedError

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
        Creates a set of 2D coordinates for the drone to follow. For now hardcoded (Random)
        Returns: Nx2 numpy array
        """
        return np.array([0, 0], [5, 5], [-5, 5], [10, 10])

        
""" 
PSEUDOCODE:
    Initialize starting position using the ground truth 
    Initialize the Eulicidian Distance from the next waypoint coordinate
    (Need something to intialize direction of the goal)
    (Find)
    State machine:  

"""
        