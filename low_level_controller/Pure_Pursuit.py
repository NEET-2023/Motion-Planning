from matplotlib.pyplot import close
import rospy
import numpy as np
import time
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32

#NEW
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PurePursuit():
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self, trajectory, odom, pose):
        self.fly_cmd = Twist()
        self.lookahead = 1.5
        self.speed = 1
        self.trajectory  = trajectory
        self.odom = odom
        self.waypoint_segments = None

        #Publish Topics
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Published Topics for Debugging Purposes
        self.point_pub = rospy.Publisher("/closest_trajectory_point", PointStamped, queue_size=1)
        self.curr_pose_pub = rospy.Publisher("/guess_pose", PointStamped, queue_size=1)
        self.stop = False

        # Error Publisher for debugging
        self.real_pose_sub = rospy.Subscriber("/real_pose", Odometry, self.realPose_callback, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=10)



    # Compute Euclidean distance between 2 points
    def distance(self, point1, point2):
        return np.linalg.norm(point2-point1)

    def minimum_distance(self, point):
        def helper(segment):
            start, end = np.array([segment[0], segment[1]]), np.array([segment[2],segment[3]])

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

    def initialize_twist_data(self):
        self.fly_cmd.linear.x=0
        self.fly_cmd.linear.y=0
        self.fly_cmd.linear.z=1
        return drive_data

    def createPoint(self, point):
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = "/map"
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        return point_msg

    def pose_update_callback(self, raw_pose):
        '''
        runs pure pursuit
        inp: pose (pose obj)
        '''
        if self.waypoint_segments is None:
            #no waypoints/trajectory to follow
            return

        if self.stop: 
            self.vel_pub.publish(self.create_stop_msg())
            return

        euler = euler_from_quaternion([raw_pose.orientation.x, raw_pose.orientation.y, raw_pose.orientation.z, raw_pose.orientation.w])
        pose = [ raw_pose.position.x, raw_pose.position.y, euler[-1]]

        #obtain waypoint segments from waypoint trajectory dots
        self.waypoint_segments = np.array([[self.trajectory[i][0], self.trajectory[i][1], self.trajectory[i+1][0], self.trajectory[i+1][1]] for i in range(len(self.trajectory.points)-1)])

        # FOR DUBUGGING: current robot position
        point = np.array(pose[:-1])
        self.curr_pose_pub.publish(self.createPoint(point))

        # compute minimum distances from robot to each segment of the trajectory
        min_distances = np.apply_along_axis(self.minimum_distance(point), 1, self.waypoint_segments)
        min_index = np.argmin(min_distances)

        forward_trajectory = self.waypoint_segments[min_index:][::-1]
        desired_point = self.waypoint_segments[min_index][2:] # default desired point as endpoint of closest line segment of trajectory

        for i, segment in enumerate(forward_trajectory):
            # start and end points of each segment 
            start, end = np.array([segment[0], segment[1]]), np.array([segment[2],segment[3]])

            V = end - start  # Vector along line segment

            a = np.dot(V,V)
            b = 2 * np.dot(V, start - point)
            c = np.dot(start,start) + np.dot(point,point) - 2 * np.dot(start,point) - self.lookahead**2

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
            min_index += len(forward_trajectory) - i - 1
            break

        #FOR DEBUGGING
        self.point_pub.publish(self.createPoint(desired_point))
	
        path_vector = desired_point - point
        new_yaw = np.arctan2(path_vector[1],path_vector[0])
        eta = new_yaw - pose[-1] 

        #obtain x and y components of eta to command movement
        self.fly_cmd.linear.x = V*np.cos(eta)
        self.fly_cmd.linear.y = V*np.sin(eta)

        self.vel_pub.publish(self.fly_cmd)

        self.waypoint_segments = self.waypoint_segments[min_index:]

        if (len(self.waypoint_segments)==1 and self.distance(point, self.waypoint_segments[-1][2:]) < self.lookahead):
            self.drive_pub.publish(self.create_stop_msg())
            self.stop = True

        if eta < (np.pi/6): self.lookahead = 1.5
        else: self.lookahead = 0.5

    def create_stop_msg(self):
        self.fly_cmd.linear.x=0
        self.fly_cmd.linear.y=0
        self.fly_cmd.linear.z=0
        return self.fly_cmd

    def realPose_callback(self, msg):
        if self.waypoint_segments is None or self.stop:
            return

        real_pose = msg.pose.pose.position
        real_pose_position = [real_pose.x, real_pose.y]
        rospy.loginfo("%s", np.array([list(point) for point in self.trajectory.points]))
        distances = np.apply_along_axis(lambda x: self.distance(x, real_pose_position), 1, np.array([list(point) for point in self.trajectory.points]))
        min_distance_index = np.argmin(distances)
        self.error_pub.publish(min(distances))

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()

    # Testing minimum_distance method
    # start = {'x':-2,'y':2}
    # end = {'x':2, 'y':2}
    # point = {'x': 0, 'y':0}
    # print(pf.minimum_distance(start,end,point))