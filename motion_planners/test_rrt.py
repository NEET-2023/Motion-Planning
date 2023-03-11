import numpy as np
import cv2
from AStar_planner import Navigator

if __name__ == "__main__":
    path = '../occupancy_grids/images/rolling_hills_map_10.png'
    occupancy_image = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
    xmin, xmax = -60, 60
    ymin, ymax = -60, 60

    NAV = Navigator(occupancy_image,xmin,xmax,ymin,ymax,[])
    NAV.min_x, NAV.max_x = xmin, xmax
    NAV.min_y, NAV.max_y = ymin, ymax
    NAV.occupancy_grid = occupancy_image
    NAV.max_row, NAV.max_col = np.array(NAV.occupancy_grid.shape) - 1

    # set the waypoints
    NAV.rrt_planner(np.array([30, 30, 10]), np.array([0, 0, 10]))
    print(NAV.path)
