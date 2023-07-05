#!/usr/bin/env python3

import rospy
import rosbag
import cv2
import time
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

##### USE Settings #####
subscribeLidar = True
subscribeWebcam = True

savePCL = False
showPlot = True
showImage = True


##### General Settings #####

# Creates a 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Create a empty list to save the points
points = []

## Counter used t
contSave = 0

##############################

##### Callbacks #####
def callback_lidar(msg):
    global points, contSave, savePCL, showPlot

    ## If the save is defined
    if savePCL == True:
        bag = rosbag.Bag(f'point_cloud{contSave}.bag', 'w')
        bag.write('/rslidar_points', msg)
        bag.close()
        contSave += 1


    if showPlot == True:
        # Extect the relevant points of the PointCloud2 message
        point_cloud = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convet the points to a numpy array
        points = np.array(list(point_cloud))


def callback_webcam(msg):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    cv2.imshow('Webcam', frame)
    cv2.waitKey(1)
##############################

#### Auxiliar functions #####

# Function to update the plot
def update_plot(frame):
    global points

    # Clean the grapth
    ax.cla()

    # Show the clound point updated
    if len(points) > 0:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2])

    # Additional graph settings
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Cloud Point')


if __name__ == '__main__':
    rospy.init_node('main', anonymous=True)
    
    if subscribeLidar == True:
        rospy.Subscriber('/rslidar_points', PointCloud2, callback_lidar)

    if subscribeWebcam == True:
        rospy.Subscriber('webcam_image', Image, callback_webcam)

    if showPlot:
        # Update continualy the graph
        ani = FuncAnimation(fig, update_plot, interval=100)
        # Show the graph
        plt.show()


    if showImage == True:
        # Wait for a moment before to create the window
        time.sleep(1)

        cv2.namedWindow('Webcam', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Webcam', 640, 480)
        
    ## Keep the node alive
    rospy.spin()

    if showImage == True:
        cv2.destroyAllWindows()






