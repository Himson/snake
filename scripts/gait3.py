#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import numpy as np
import cv2 as cv
import sensor_msgs.point_cloud2
import string

def talker():
     module_num = 17
     joint_num = 16
     pub_joints = []
     for i in range(1,joint_num+1):
	 	topic = '/wl_snake/joint_%d/cmd_pos'%i
	 	pub = rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=10)
	 	pub_joints.append(pub)
     #sub = rospy.Subscriber('/order',std_msgs.msg.String,cb)
     rospy.init_node('controller', anonymous=True)
     rate = rospy.Rate(100) # 100hz
     while not rospy.is_shutdown():
         now = rospy.get_rostime()
         t = rospy.Time(now.secs, now.nsecs).to_sec()
         head_amp	= 45*np.pi/180.0
         head_omega	= 0.25*np.pi
         for i in range(1, module_num):
			if i == 1:
				angle = 0
				pub_joints[i-1].publish(angle)
			if i == 2:
				angle = -70 / 180 * np.pi
				pub_joints[i-1].publish(angle)
			if i == 3:
				angle = head_amp*np.sin(head_omega*t) + 70/180*np.pi
				pub_joints[i-1].publish(angle)                
			if i > 3 and i%2 == 1:
				angle = 70
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i > 2 and i%2 == 0:
				angle = 0.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
         rate.sleep()

if __name__ == '__main__':
     try:		
         talker()
     except rospy.ROSInterruptException:
         pass