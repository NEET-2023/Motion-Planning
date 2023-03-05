import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

class dummy():
    def __init__(self):
        self.sub = rospy.Subscriber('/map2d', OccupancyGrid, self.callback)

    def callback(self, msg):
        occupancy = np.array(msg.data)
        occupancy = occupancy.reshape((240, 240))
        np.where(occupancy == 0, 255, occupancy)
        np.where(occupancy == 100, 0, occupancy)
        np.where(occupancy == -1, 120, occupancy)
        occupancy = occupancy.astype('uint8')

        grey_image = cv2.cvtColor(occupancy, cv2.COLOR_GRAY2BGR)
        cv2.imshow("name", grey_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        cv2.imwrite("occupancy_grid_image.png", grey_image)

if __name__ == '__main__':
    try:
        rospy.init_node("dummy")
        dummy = dummy()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
