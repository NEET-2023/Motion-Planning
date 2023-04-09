import rospy
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class PlaceSensor():
    def __init__(self, rate):
        # Height control information
        self.descent_height = 2
        self.flight_height = 10
        self.fly_cmd = Twist()

        # ROS relevant initializations
        self.finished_sub = rospy.Subscriber('/done_travelling', Bool, self.landing_callback)
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.placed_pub = rospy.Publisher('/sensor_placed', Bool, queue_size=1)
        self.rate = rate

    def landing_callback(self, finished: Bool) -> None:
        """
        Controller to make the drone face in the direction of the next waypoint

        Parameters:
        finised (Bool): a boolean flag which tells us if we made it to the waypoint

        Returns: 
        None
        """

        if finished.data:
            self.land_drone()
    
    def odom_callback(self, msg: Odometry) -> None:
        """
        Simply takes in the odometry message and extracts the pose

        Parameters:
        msg (Odometry): contains odometry information

        Returns:
        None
        """
        self.pose = msg.pose.pose

    def range_callback(self, msg) -> None:
        """
        Provides the distance to ground information from the 1D LIDAR

        Parameters:
        msg (Range): Range object containing distance to ground data

        Returns:
        None
        """
        self.ground_dist = msg.range

    def land_drone(self) -> None:
        """
        Controls the drone to move down towards a specified height off the ground. 
        Once done, it indicates so by publishiing placed_msg

        Parameters:
        None

        Returns:
        None
        """
        # determine where the drone is currently located
        # location = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])
        # within_threshold = abs(location[2] - self.descent_height) < 0.1
        within_threshold = abs(self.ground_dist - self.descent_height) < 0.1
        if not within_threshold:
            print("Going down")
        
        # bring the drone down to the sensor drop height
        self.fly_cmd.linear.x = 0
        self.fly_cmd.linear.y = 0
        self.fly_cmd.linear.z = 0 if within_threshold else np.sign(self.descent_height - self.ground_dist)
        self.vel_pub.publish(self.fly_cmd)
        
        # condition to have the drone hover at the dropoff spot to simulate sensor placement
        if within_threshold:
            placed_msg = Bool()
            placed_msg.data = True
            self.placed_pub.publish(placed_msg)
            print("Placed Sensor")
            self.rate.sleep()

if __name__ ==  "__main__":
    try:
        rospy.init_node('Place_Sensor', anonymous=True)
        rate = rospy.Rate(0.5)
        PS = PlaceSensor(rate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass