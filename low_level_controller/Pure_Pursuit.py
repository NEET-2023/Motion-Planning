import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point32, Twist
from tf.transformations import euler_from_quaternion

class PurePursuit():
    """ 
    Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self, path=None, odom=None, pose=None, threshold=0):
        self.fly_cmd = Twist()
        self.lookahead = 3
        self.speed = 1.5
        self.path = path
        self.path_segments = None
        self.full_path_segments = None
        self.pose = pose
        # PD Controller Initializations
        self.kp = 1
        self.kd = 0
        self.prev_v_world = np.array([0, 0, 0])
        # controller constraints
        self.threshold = threshold
        self.ground_dist = 0
        self.within_threshold = False

        # Controller type
        self.type = "PP"

        # Published Topics for Debugging Purposes
        self.pursuit_point_pub = rospy.Publisher("/pursuit_point", PointStamped, queue_size=1)
        self.pose_pub = rospy.Publisher("/pp_pose", Point32, queue_size=1)
        self.stop = False

        #waypoint constraints
        self.waypoint_threshold = 0.3 # how close we need to be to the waypoint
        self.pursuit_epsilon = 0.1 #allows us to satisfy |x_f - x_d| < waypoint_threshold

        # error measuring publisher
        self.error_pub = rospy.Publisher("/pp_error", Float32, queue_size=1)
        self.max_error_pub = rospy.Publisher("/pp_max_error", Float32, queue_size=1)
        self.max_min_distance = 0
        self.min_distance = 0

    # Compute Euclidean distance between 2 points
    def distance(self, point1, point2):
        return np.linalg.norm(point2 - point1)

    def minimum_distance(self, point):
        def helper(segment):
            start, end = np.array([segment[0], segment[1]]), np.array([segment[2], segment[3]])

            # Return minimum distance between line segment (start-end) and point p
            l2 = self.distance(start, end)**2
            if (l2 == 0.0):
                return self.distance(point, start)


            # v = start, w = end
            # Consider the line extending the segment, parameterized as v + t (w - v).
            # We find projection of point p onto the line.
            # It falls where t = [(p-v) . (w-v)] / |w-v|^2
            # We clamp t from [0,1] to handle points outside the segment vw.

            t = max(0, min(1, np.dot(point - start, end - start) / l2))
            projection = start + t * (end - start)  # Projection falls on the segment
            return self.distance(point, projection)
        return helper

    def createPoint(self, point):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "world"
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = 10
        return point_msg

    def actuate(self):
        """
        Runs the pure pursuit algorithm to determine the necessary velocity commands in the world frame
        
        Parameters:
        None

        Returns:
        v_cmd (np.ndarray): the velocity command in the world frame
        """
        if self.path_segments is None:
            #no waypoints/trajectory to follow
            print('No Trajectory')
            return

        if self.stop: 
            return True, np.array([0, 0, 0])

        #DETERMINE CURRENT LOCATION OF DRONE
        euler = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        pose = [self.pose.position.x, self.pose.position.y, euler[2]]

        #obtain waypoint segments from waypoint trajectory dots
        # self.path_segments = np.array([[self.path[i][0], self.path[i][1], self.path[i+1][0], self.path[i+1][1]] for i in range(len(self.path) - 1)])

        point = np.array(pose[:-1])
        pub_point = Point32()
        pub_point.x = point[0]
        pub_point.y = point[1]
        self.pose_pub.publish(pub_point)

        #FIND PATH POINT CLOSEST TO DRONE
        # compute minimum distances from robot to each segment of the trajectory
        min_distances = np.apply_along_axis(self.minimum_distance(point), 1, self.path_segments)
        full_min_distances = np.apply_along_axis(self.minimum_distance(point), 1, self.full_path_segments)
        min_index = np.argmin(min_distances)

        # no need to look ahead of only one path segment left, just to to the end
        desired_point = self.path_segments[min_index][2:]

        # modulate speed if getting close to the waypoint
        self.speed = 1.5 if len(self.path_segments) > 5 else 0.5

        if len(self.path_segments) != 1:
            forward_path = self.path_segments[min_index:][::-1]

            for i, segment in enumerate(forward_path):
                # start and end points of each segment 
                start, end = np.array([segment[0], segment[1]]), np.array([segment[2],segment[3]])

                V = end - start  # Vector along line segment

                a = np.dot(V,V)
                b = 2 * np.dot(V, start - point)
                c = np.dot(start, start) + np.dot(point, point) - 2 * np.dot(start, point) - self.lookahead**2

                # Discriminant
                disc = b**2 - 4*a*c

                if disc < 0: # no real solutions
                    continue 

                sqrt_disc = np.sqrt(disc)
                t1 = (-b + sqrt_disc) / (2 * a)
                t2 = (-b - sqrt_disc) / (2 * a)

                if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
                    continue

                t = max(0, min(1, - b / (2 * a)))
                desired_point = start + t * V
                min_index += len(forward_path) - i - 1
                break

        #FOR DEBUGGING
        self.pursuit_point_pub.publish(self.createPoint(desired_point))
        path_vector = desired_point - point
        new_yaw = np.arctan2(path_vector[1],path_vector[0])
        eta = new_yaw

        #UPDATE DRONE POSITION 
        # obtain x and y components of eta to command movement
        # hard code flight speed for now (1)
        v_x = self.speed*np.cos(eta)
        v_y = self.speed*np.sin(eta)

        self.path_segments = self.path_segments[min_index:]

        #CHANGE LOOKAHEAD BASED ON CORNERS
        # if eta < (np.pi/6): self.lookahead = 3.0
        # else: self.lookahead = 1.5

        # If close enough to waypoint, end iteration:
        if self.distance(point, self.path_segments[-1][2:]) < self.waypoint_threshold:
            self.stop = True
            return True,np.array([0,0,0])

        '''Ground laser PD Control'''
        # if env is too close to drone
        if self.within_threshold:
            print("violated threshold")
            v_z = self.threshold-self.ground_dist
        # nominal threshold present, control global z value
        else:
            # fix hard coding later
            z_error = 10 - self.pose.position.z
            v_z = self.kp * z_error - self.kd*self.prev_v_world[2]

        # get the minimum distance from drone to path
        self.min_distance = min(full_min_distances)
        # update biggest min distance if needed
        self.max_min_distance = self.min_distance if self.min_distance > self.max_min_distance else self.max_min_distance
        # publish the latest biggest min distance
        error_data = Float32()
        max_error_data = Float32()
        error_data.data = self.min_distance
        max_error_data.data = self.max_min_distance
        self.error_pub.publish(error_data)
        self.max_error_pub.publish(max_error_data)

        return False, (v_x, v_y, v_z)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()