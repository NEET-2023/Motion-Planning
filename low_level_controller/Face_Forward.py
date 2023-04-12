import rospy
import numpy as np
from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation as R

class FaceForward():
    def __init__(self):
        self.facing_forward = False
        self.prev_angular_z = 0
        self.kp = 1
        self.kd = 0.2
        self.rotation_pub_debug = rospy.Publisher('/rotation_angle', Float32, queue_size=1)
        self.vel_pub_debug = rospy.Publisher('/vel_angle', Float32, queue_size=1)

    def face_forward_control(self, velocity: np.ndarray, pose):
        '''
        Controller to make the drone face in the direction of the next waypoint

        Parameters:
        velocity (np.ndarray): velocity command in the world frame
        pose (pose): the pose of the drone in the world

        Returns: 
        None
        '''
        quaternion = pose.orientation
        r = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # the yaw angle of the drone in the world frame
        yaw_world = r.as_euler('xyz')[2] % (2*np.pi) # put between (0, 2*pi)
        # convert the velocity into the frame of the drone
        v_drone = np.array([velocity[0]*np.cos(yaw_world) + velocity[1]*np.sin(yaw_world), 
                            -velocity[0]*np.sin(yaw_world) + velocity[1]*np.cos(yaw_world)])
        # set the x and y velocity commands in the drone frame
        vx, vy = v_drone
        vz = velocity[2]
        
        # get angle of velocity vector to vector the angles are defined by 
        a = np.array([1,0])
        # make velocity_angle pos or neg depending on cross product result
        velocity_angle = np.sign(np.cross(a, velocity[:2]))*np.arccos(np.dot(a, velocity[:2])/(np.linalg.norm(a)*np.linalg.norm(velocity[:2]))) % (2*np.pi)
        # shortest angluar distance to rotate
        # rotation_angle = velocity_angle - yaw_world
        rotation_angle = ((velocity_angle*(180/np.pi) - yaw_world*(180/np.pi) + 180) % 360 - 180) * np.pi/180

        # dubugging publishers
        self.vel_pub_debug.publish(velocity_angle)
        self.rotation_pub_debug.publish(rotation_angle)

        # set the yaw rate to realign the drone
        omega_z = (self.kp*rotation_angle - self.kd*self.prev_angular_z)
        # in case we are not moving the drone
        if np.isnan(omega_z):
            omega_z = 0
    
        # let the drone face motion direction before starting to move
        if not self.facing_forward:
            vx, vy = 0, 0

        if abs(rotation_angle) < 0.5:
            self.facing_forward = True

        # publish angle command for debugging purposes
        self.prev_angular_z = omega_z
        
        return self.facing_forward, (vx, vy, vz), omega_z
