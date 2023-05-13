import rospy
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool, Int16
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PointStamped


class PlaceSensor():
    def __init__(self, rate):
        # Height control information
        self.place_height = 2
        self.pickup_height = 0.3
        self.flight_height = 10
        self.fly_cmd = Twist()
        self.ground_dist = 0
        self.pose = None

        # pod state information
        self.gripper_location_drone = np.array([0, 0, -0.1]) # gripper relative to the drone
        self.gripper_location_world = np.array([0, 0, 0]) # gripper relative to the world
        self.pod_location_drone = np.array([0, 0, 0]) # pod relative to the drone initialization
        self.pod_location_world = np.array([0, 0, 0]) # pod relative to the world initialization
        self.pickup_point_pod = np.array([0, 0, 0.2]) # pickup point relative to the pod
        self.pod_index = 0

        # ROS state machine variables 
        self.place_sensor_sub = rospy.Subscriber('/place_sensor', Bool, self.place_callback)
        self.pickup_sensor_sub = rospy.Subscriber('/pickup_sensor', Bool, self.pickup_callback) 
        # ROS drone state information
        self.odom_sub = rospy.Subscriber('/ground_truth/state', Odometry, self.odom_callback)
        self.range_sub = rospy.Subscriber('/sonar_height', Range, self.range_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # ROS pod state information
        self.pod_location_sub = rospy.Subscriber('/pod_location', PointStamped, self.pod_location_callback)
        self.pod_index_sub = rospy.Subscriber('/pod_index', Int16, self.pod_index_callback)
        # ROS states to be published
        self.placed_pub = rospy.Publisher('/sensor_placed', Bool, queue_size=1)
        self.retrived_pub = rospy.Publisher('/sensor_pickedup', Bool, queue_size=1)

        self.rate = rate

        # directory to pod urdf
        self.directory = os.getcwd()[:-36] + '/Perception/sensor_pod'

        #placement tolerance
        self.location_tolerance = 0.05
        
        #placement flag
        self.picked = False

    def range_callback(self, msg) -> None:
        """
        Provides the distance to ground information from the 1D LIDAR

        Parameters:
        msg (Range): Range object containing distance to ground data

        Returns:
        None
        """
        self.ground_dist = msg.range

    def odom_callback(self, msg: Odometry) -> None:
        """
        Simply takes in the odometry message and extracts the pose

        Parameters:
        msg (Odometry): contains odometry information

        Returns:
        None
        """
        self.pose = msg.pose.pose

    def pod_location_callback(self, msg: PointStamped) -> None:
        """
        Callback function that stores the location of the sensor pod relative to the
        position of the drone

        Parameters:
        msg (PoseStamped): Contains pose information about the sensor pod

        Returns:
        None
        """
        self.pod_location_drone = np.array([msg.point.x, msg.point.y, msg.point.z])

    def pod_index_callback(self, msg: Int16) -> None:
        """
        """
        self.pod_index = msg.data

    def place_callback(self, msg: Bool) -> None:
        """
        Callback function that determines if we are in the state of placing a sensor
        LLC to place a sensor takes over sending velocity commands.

        Controls the drone to move directly downwards towards a specified height off the ground. 
        Once done, it indicates so by publishiing placed_msg
        ^^ PLACEHOLDER FUNCTIONALITY


        Parameters:
        msg (Bool): The boolean flag that tells us if we are to place a sensor

        Returns: 
        None
        """
        if msg.data:
            # determine where the drone is currently located
            within_threshold = abs(self.ground_dist - self.place_height) < 0.1
            if not within_threshold:
                print("Going down")
            
            # bring the drone down to the sensor drop height
            self.fly_cmd.linear.x = 0
            self.fly_cmd.linear.y = 0
            self.fly_cmd.linear.z = 0 if within_threshold else 0.5*np.sign(self.place_height - self.ground_dist)
            self.vel_pub.publish(self.fly_cmd)
            
            # condition to have the drone hover at the dropoff spot to simulate sensor placement
            if within_threshold:
                placed_msg = Bool()
                placed_msg.data = True
                x = self.pose.position.x
                y = self.pose.position.y
                z = self.pose.position.z - self.ground_dist + 1.5
                sensor = f"sensor_{self.pod_index}"
                os.system(f"rosrun gazebo_ros spawn_model -file {self.directory}/sensor_pod.urdf -urdf -x {x} -y {y} -z {z} -model {sensor}")
                print(f"Placed Sensor: {sensor}")
                self.placed_pub.publish(placed_msg)

    def pickup_callback(self, msg: Bool) -> None:
        """
        Callback function that determines if we are in the state of picking up a sensor
        LLC to pickup a sensor takes over sending velocity commands.

        Determines the locaion of the sensor pod in world coordinates and actuates the
        gripper to a pickup location relative to the pod.

        Parameters:
        msg (Bool): The boolean flag that tells us if we are to pickup a sensor

        Returns: 
        None
        """

        if msg.data:
            # determine if we are close enough to the pod
            within_threshold = self.ground_dist - self.pickup_height < 0.1
            # bring the drone down to the sensor pickup height
            if not within_threshold:
                # at home base, just go straight down to pretend pick up pod
                if np.any(np.isnan(self.pod_location_drone)):
                    print("Going down, at base")
                    self.fly_cmd.linear.x = 0
                    self.fly_cmd.linear.y = 0
                    self.fly_cmd.linear.z = 0 if within_threshold else np.sign(self.pickup_height - self.ground_dist)
                # at pod location, use perception information to align drone with pod
                else:
                    print("Going down, at pod location")
                    deviation = self.pod_location_drone[:2]
                    self.fly_cmd.linear.x = deviation[1]**(1/12)
                    self.fly_cmd.linear.y = deviation[2]**(1/12)
                    self.fly_cmd.linear.z = 0 if within_threshold else 0.25*np.sign(self.pickup_height - self.ground_dist)
                self.vel_pub.publish(self.fly_cmd)
            # condition to have the drone hover at the dropoff spot to simulate sensor placement
            if within_threshold:
                picked_msg = Bool()
                picked_msg.data = True
                self.fly_cmd.linear.x = 0
                self.fly_cmd.linear.y = 0
                self.fly_cmd.linear.z = 0
                self.vel_pub.publish(self.fly_cmd)
                sensor = f"sensor_{self.pod_index}"
                os.system(f"rosservice call gazebo/delete_model '{sensor}'")
                print(f"Picked up Sensor: {sensor}")
                self.retrived_pub.publish(picked_msg)

if __name__ ==  "__main__":
    try:
        rospy.init_node('Place_Sensor', anonymous=True)
        rate = rospy.Rate(0.5)
        PS = PlaceSensor(rate)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass