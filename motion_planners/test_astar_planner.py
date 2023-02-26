import numpy as np
from AStar_planner import Navigator

if __name__ == "__main__":
    test_height = np.zeros((40, 40))
    test_occupancy = np.array([[1 if np.random.random() < 0.2 else 0 for i in range(40)] for j in range(40)])

    NAV = Navigator()
    NAV.min_x, NAV.max_x = -10, 30
    NAV.min_y, NAV.max_y = -10, 20
    NAV.height_map = test_height
    NAV.occupancy_grid = test_occupancy
    NAV.max_row, NAV.max_col = np.array(NAV.occupancy_grid.shape) - 1
    start = np.array([-10, 20])
    goal = np.array([30, -10])
    NAV.a_star_planner(goal=goal, start=start, debug=False)
    print(NAV.path)
