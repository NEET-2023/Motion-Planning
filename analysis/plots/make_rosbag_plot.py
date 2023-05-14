import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

if __name__ == '__main__':
    error_path_waypoint1 = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/first_waypoint_pp_error.csv'
    max_error_path_waypoint1 = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/first_waypoint_pp_max_error.csv'
    error_path_waypoint2 = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/second_waypoint_pp_error.csv'
    max_error_path_waypoint2 = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/waypoint_navigation/second_waypoint_pp_max_error.csv'

    # get the data from for both waypoints
    columns = ["time", "field_data"]
    df_path_error1 = pd.read_csv(error_path_waypoint1, usecols=columns)
    df_path_max_error1 = pd.read_csv(max_error_path_waypoint1, usecols=columns)
    df_path_error2 = pd.read_csv(error_path_waypoint2, usecols=columns)
    df_path_max_error2 = pd.read_csv(max_error_path_waypoint2, usecols=columns)

    # get time data
    time_initial1 = df_path_error1.time[0]
    time1 = (np.array(df_path_error1.time) - time_initial1)/1e9
    time_initial2 = df_path_error2.time[0]
    time2 = (np.array(df_path_error2.time) - time_initial2)/1e9

    # get deviation data
    path_deviation1 = df_path_error1.field_data
    path_max_deviation1 = df_path_max_error1.field_data
    path_deviation2 = df_path_error2.field_data
    path_max_deviation2 = df_path_max_error2.field_data


    fig1 = plt.figure()
    plt.title("Pure Pursuit Performance: Waypoint 1")
    plt.plot(time1, path_deviation1, label="Path Deviation [m]")
    plt.plot(time1, path_max_deviation1, label="Maximum Deviation [m]")
    plt.legend()
    plt.savefig('PP_waypoint1')

    fig2 = plt.figure()
    plt.title("Pure Pursuit Performance: Waypoint 2")
    plt.plot(time2, path_deviation2, label="Path Deviation [m]")
    plt.plot(time2, path_max_deviation2, label="Maximum Deviation [m]")
    plt.legend()
    plt.savefig('PP_waypoint2')
