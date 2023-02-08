import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class WaypointFollower():
    def __init__(self):
        self.waypoints = np.array([[0, 0, 1], [1, -1, 5], [2, -4, 1], [-2, -1, 1]])
        self.threshold = 0.1
        self.waypoint_index = 0
        self.kp = 0.8
        self.kd = 0.6
        self.max_v = 2
        self.prev_v = np.array([0, 0, 0])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.init_node('waypointsFollower', anonymous=True)
        rospy.Subscriber('/ground_truth/state', Odometry, self.callback)

    def callback(self, msg):
        # extrac the current location of the drone
        loc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

        # determine if we are at goal to update waypoint
        if self.at_goal(loc):
            self.waypoint_index += 1
            self.waypoint_index %= len(self.waypoints)
        waypoint = self.waypoints[self.waypoint_index]

        # PD controller
        error = waypoint - loc
        v = self.kp*error - self.kd*self.prev_v
        self.prev_v = v

        # create cmd_velocity object
        cmd_vel = Twist()
        cmd_vel.linear.x = min(v[0], self.max_v)
        cmd_vel.linear.y = min(v[1], self.max_v)
        cmd_vel.linear.z = min(v[2], self.max_v)

        # publish message
        self.pub.publish(cmd_vel)

    def at_goal(self, loc):
        curr_waypoint = np.array(self.waypoints[self.waypoint_index])
        return np.linalg.norm(loc - curr_waypoint) < self.threshold


if __name__ == '__main__':
    try:
        WF = WaypointFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
