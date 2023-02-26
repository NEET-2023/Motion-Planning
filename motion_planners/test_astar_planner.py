import numpy as np
from AStar_planner import Navigator

if __name__ == "__main__":
    test_height = np.zeros((40, 40))
    test_occupancy = np.array([[1 if np.random.random() < 0.2 else 0 for i in range(40)] for j in range(40)])

    NAV = Navigator()
    NAV.min_x, NAV.max_x = -10, 10
    NAV.min_y, NAV.max_y = -10, 10
    NAV.height_map = test_height
    NAV.occupancy_grid = test_occupancy

    goal = np.array([1, 2])
    start = np.array([39, 24])
    NAV.a_star_planner(goal=goal, start=start, debug=True)
    print(NAV.path)
