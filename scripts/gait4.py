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
     
     while not rospy.is_shutdown():
         now = rospy.get_rostime()
         t = rospy.Time(now.secs, now.nsecs).to_sec()
         key = getKey()
         angle = np.zeros(joint_num)
         flag  = True
         if t < 100:
            amp		= 15.0*np.pi/180.0
            phase_diff 	= 0.5*np.pi
            omega 		= np.pi
            direction 	= 1 # 1 or -1
         
            for i in range(joint_num):
			    angle = amp*np.sin(omega*t*direction+phase_diff*(i))
         		#rospy.loginfo(angle)
			    pub_joints[i].publish(angle)
         if t > 100:	
          for i in range(module_num-1):
	  	#angle = -amp*np.sin(omega+phase_diff*(i-1))
	 	#pub_joints[i-1].publish(angle)
            angle = amp*np.sin(omega+phase_diff*(i-1))
            pub_joints[i].publish(angle)
         if t > 140:
            for i in range(module_num-1):
                angle = np.pi/2 * np.sin(-omega*t+phase_diff*i)  
                pub_joints[0].publish(angle)    
         rate.sleep()

if __name__ == '__main__':
     try:		
         talker()
     except rospy.ROSInterruptException:
         pass