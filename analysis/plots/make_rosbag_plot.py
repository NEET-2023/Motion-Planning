import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

if __name__ == '__main__':
    error_path = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/first_waypoint_pp_error.csv'
    max_error_path = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/first_waypoint_pp_max_error.csv'

    columns = ["time", "field_data"]
    df_path_error = pd.read_csv(error_path, usecols=columns)
    time_initial = df_path_error.time[0]
    time = (np.array(df_path_error.time) - time_initial)/1e9
    path_deviation = df_path_error.field_data
    plt.title("Pure Pursuit Performance")
    plt.plot(time, path_deviation, label="Path Deviation [m]")
    # plt.plot(time deviation, label="Maximum Deviation [m]")
    plt.legend()
    plt.savefig('PP_waypoint1')
    # plt.show()
