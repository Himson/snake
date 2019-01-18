import rospy
import std_msgs.msg
import numpy as np
import cv2 as cv
import sensor_msgs.msg._PointCloud2 as pc2

def callback(data):
    rospy.loginfo("I am listening")   


def displaydepth():
    rospy.init_node('displaydepth', anonymous=True)
    rospy.Subscriber("/camera/depth/points",pc2.PointCloud2, callback)
    cv.namedWindow("pointcloud",0)
    rospy.spin()
    cv.destroyAllWindows()

if __name__ == '__main__':
     try:
         displaydepth()
     except rospy.ROSInterruptException:
         pass
