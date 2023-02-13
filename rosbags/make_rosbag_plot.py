import matplotlib.pyplot as plt
import pandas as pd

if __name__ == '__main__':
    columns = ["%time", "field.header.seq", "field.header.stamp", "field.header.frame_id", "field.radiation_type", "field.field_of_view", "field.min_range", "field.max_range", "field_range"]
    df = pd.read_csv("sonar_height.csv", usecols=columns)
    plt.title("Naive Controller Performance")
    plt.plot(df.field_range, label="Sonar Reading [m]")
    plt.plot(1 - df.field_range, label="Ground Height Error [m]")
    plt.legend()
    plt.show()
