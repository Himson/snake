#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import numpy as np
import cv2 as cv
import sensor_msgs.point_cloud2
import string

import tty
import select
import sys
import termios
settings = termios.tcgetattr(sys.stdin)
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

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
     amp		= 15.0*np.pi/180.0
     phase_diff 	= 0.5*np.pi
     omega 		= np.pi
     angle = np.zeros(joint_num)
     while not rospy.is_shutdown():
         now = rospy.get_rostime()
         t = rospy.Time(now.secs, now.nsecs).to_sec()
         key = getKey()
         flag  = True
         if key == 'q':
             exit()
         if key == '-':
             flag = ~flag
             rospy.loginfo(flag)
         if key == 'a':
             if flag == True:
                angle[0] += 0.1
             if flag == False:
                angle[0] -= 0.1
         if t < 10:	
          for i in range(joint_num):
            angle[i] = amp*np.sin(omega*t+phase_diff*(i))
            
         for i in range(joint_num): 
            pub_joints[i].publish(angle[i]) 
            print(angle[i])   
         rate.sleep()

if __name__ == '__main__':
     try:		
         talker()
     except rospy.ROSInterruptException:
         pass