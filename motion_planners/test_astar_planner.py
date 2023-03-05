import numpy as np
import cv2
from AStar_planner import Navigator

if __name__ == "__main__":
    path = '../occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    xmin, xmax = -60, 60
    ymin, ymax = -60, 60

    test_height = np.zeros((2000, 2000))

    NAV = Navigator()
    NAV.min_x, NAV.max_x = xmin, xmax
    NAV.min_y, NAV.max_y = ymin, ymax
    NAV.height_map = test_height
    NAV.occupancy_grid = occupancy_image
    NAV.max_row, NAV.max_col = np.array(NAV.occupancy_grid.shape) - 1

    # set the waypoints
    free_grid = np.transpose(np.where(occupancy_image == 0))
    rows = np.random.choice(free_grid.shape[0], 5, replace=False)
    waypoints_grid = free_grid[rows, :]
    waypoints = []
    NAV.waypoints = waypoints

    # print(NAV.grid_to_meters(1999, 1999))
    # print(NAV.meters_to_grid(-100, -100))

    # NAV.waypoints = [np.array([-5, -7, 2]), np.array([0, 0, 2]), np.array([30, 15, 2])]
    # NAV.debug_location = np.array([2, 2, 2])
    # NAV.debug = True
    # while True:
    #     NAV.odom_callback(NAV.debug_location)
    #     if NAV.done_travelling or not NAV.path_found: 
    #         break

