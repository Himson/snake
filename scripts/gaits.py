#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
import numpy as np
 
def talker():
     module_num= 17
     joint_num = 16
     pub_joints= []
     for i in range(1,joint_num+1):
	topic = '/wl_snake/joint_%d/cmd_pos'%i
	pub = rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=10)
	pub_joints.append(pub)
     
     rospy.init_node('controller', anonymous=True)
     rate = rospy.Rate(100) # 100hz
     while not rospy.is_shutdown():
	 now = rospy.get_rostime()
	 t = rospy.Time(now.secs, now.nsecs).to_sec()	
	 #rospy.loginfo("Current time %f", t)

	 gait_type = 'rolling' # rolling, slithering, scanning

	 if gait_type == 'rolling':
         	amp		= 15.0*np.pi/180.0
	 	phase_diff 	= 0.5*np.pi
	 	omega 		= np.pi
		offset		= 0.0
		direction 	= 1 # 1 or -1
	 	z 		= 0.0
		y 		= 1-z
         
	 	for i in range(1,joint_num+1):
			angle = amp*np.sin(omega*t*direction+phase_diff*(i-1))
         		#rospy.loginfo(angle)
         		pub_joints[i-1].publish(angle)
	 elif gait_type == 'slithering':
		# spatial frequency		
		spat_freq_o = 6.0/float(module_num)*np.pi
		spat_freq_e = 12.0/float(module_num)*np.pi

		#temporal frequency
		temp_freq_o = 1.5/np.pi
		temp_freq_e = 3.0/np.pi

		# temporal phase offset between horizontal and vertical waves
        	TPO = -90.0*np.pi/180.0

        	# amplitude
        	A_o = 70.0*np.pi/180.0
		A_e = 10.0*np.pi/180.0

		# offset
				
		C_e = 0.0*np.pi/180.0

		#linear coefficient
		z = 0.3
		y = 1-z
		#z_o = 0.3
		#y_o = 1-z_o
		#z_e = 0.75
		#y_e = 1-z_e

		# direction (1 or -1)
        	direction   = 1	
		
		for i in range(1, joint_num+1):
			if i%2 == 1:
				angle_o = ((i-1)/float(module_num)*y+z)*A_o*np.sin( 2.0*np.pi*temp_freq_o*direction*t + i*spat_freq_o + TPO) + C_o
				#rospy.loginfo(angle_o)
				pub_joints[i-1].publish(angle_o)
			else:
				angle_e = ((i-1)/float(module_num)*y+z)*A_e*np.sin(2.0*np.pi*temp_freq_e*direction*t + i*spat_freq_e) + C_e
				#rospy.loginfo(angle_e)
				pub_joints[i-1].publish(angle_e)
	 elif gait_type == 'scanning':
		head_amp	= 50.0*np.pi/180.0
		head_omega	= 0.25*np.pi
		for i in range(1, module_num):
			if i == 1:
				angle = head_amp*np.sin(head_omega*t)
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i == 2:
				angle = 80.0*np.pi/180.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i == 3:
				angle = 0.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i == 4:
				angle = 80.0*np.pi/180.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i == 5:
				angle = 30.0*np.pi/180.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i > 5 and i%2 == 1:
				angle = 80.0*np.pi/180.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
			if i > 5 and i%2 == 0:
				angle = 0.0
				#rospy.loginfo(angle)
				pub_joints[i-1].publish(angle)
	 else:
		pass
         rate.sleep()
 
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass
