import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

if __name__ == '__main__':
    csv_path = '~/catkin_ws/src/Motion-Planning/analysis/rosbags/naive_A_to_B_Controller/sonar_height.csv'

    columns = ["%time", "field.header.seq", "field.header.stamp", "field.header.frame_id", "field.radiation_type", "field.field_of_view", "field.min_range", "field.max_range", "field_range"]
    df = pd.read_csv(csv_path, usecols=columns)
    plt.title("Naive Controller Performance")
    plt.plot(df.field_range, label="Sonar Reading [m]")
    plt.plot(1 - df.field_range, label="Ground Height Error [m]")
    plt.plot(np.arange(0, 487), np.ones(487), label="Target Height [m]")
    plt.legend()
    plt.savefig('naive_A_to_B_performance')
    plt.show()
