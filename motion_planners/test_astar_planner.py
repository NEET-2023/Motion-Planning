import numpy as np
from AStar_planner import Navigator

if __name__ == "__main__":
    test_height = np.zeros((200, 200))
    test_occupancy = np.array([[1 if np.random.random() < .001 else 0 for i in range(200)] for j in range(200)])

    NAV = Navigator()
    NAV.min_x, NAV.max_x = -10, 30
    NAV.min_y, NAV.max_y = -10, 20
    NAV.height_map = test_height
    NAV.occupancy_grid = test_occupancy
    NAV.max_row, NAV.max_col = np.array(NAV.occupancy_grid.shape) - 1

    NAV.waypoints = [np.array([-5, -7, 2]), np.array([0, 0, 2]), np.array([30, 15, 2])]
    NAV.debug_location = np.array([2, 2, 2])
    NAV.debug = True
    while True:
        NAV.odom_callback(NAV.debug_location)
        if NAV.done_travelling or not NAV.path_found: 
            break

