import numpy as np

class PD():
    def __init__(self, path=None, pose=None, threshold=0):
        # Path Information
        self.path = path
        self.pose = pose
        self.path_index = 0
        # PD Controller Initializations
        self.kp = 1
        self.kd = 0
        self.prev_v_world = np.array([0, 0, 0])
        # controller constraints
        self.threshold = threshold
        self.ground_dist = 0
        self.within_threshold = False
        # controller type
        self.type = "PD"

    def actuate(self):
        # Extract the pose from the provided class variable
        location = np.array([self.pose.position.x, self.pose.position.y, self.pose.position.z])

        # Check whether we are close enough to the next point in the path
        if np.all(np.absolute(self.path[self.path_index] - location) < np.array([0.2, 0.2, 0.2])):
            self.path_index += 1
            # We are at the end of the path, indicate so and return 0 veloity command
            if self.path_index == len(self.path):
                return True, np.array([0 ,0, 0])
        # next location within the path to reach 
        path_target = self.path[self.path_index] 
        # print(f"path target: {path_target}")


        # pass in velocity commands to move drone, implement x, y value PD
        errors = path_target - location 
        # calculate velocity vector with respect to the world frame
        vx_world, vy_world = self.kp*errors[:2] - self.kd*self.prev_v_world[:2]

        #if env is too close to drone
        if self.within_threshold:
            print('violated threshold')
            v_z = self.threshold-self.ground_dist
        # nominal threshold present, control global z value
        else:
            z_error = path_target[2] - location[2]
            v_z = self.kp * z_error - self.kd*self.prev_v_world[2]
        return False, np.array((vx_world, vy_world, v_z))
