import numpy as np

class Node():
    def __init__(self, location, parent=None):
        self.parent = parent
        self.location = location
    
    def get_vec(self, dest):
        vec = dest - self.location
        return vec/np.sqrt(np.sum(np.square(vec)))

    def get_dist(self, point):
        return np.linalg.norm(point - self.location)
