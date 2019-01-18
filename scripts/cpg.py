#!/usr/bin/env python
# license removed for brevity
import rospy
import std_msgs.msg
 
def talker():
     pub_amp = rospy.Publisher('/wl_snake/amplitude', std_msgs.msg.Float64, queue_size=10)
     pub_vel = rospy.Publisher('/wl_snake/vel', std_msgs.msg.Float64, queue_size=10)
     pub_the = rospy.Publisher('/wl_snake/phase_diff', std_msgs.msg.Float64, queue_size=10)
     pub_red = rospy.Publisher('/wl_snake/lin_red', std_msgs.msg.Float64, queue_size=10)
     pub_rad = rospy.Publisher('/wl_snake/radius', std_msgs.msg.Float64, queue_size=10)
     
     rospy.init_node('controller', anonymous=True)
     rate = rospy.Rate(100) # 100hz
     while not rospy.is_shutdown():
         a = 0.5
	 t = 1.5708
	 w = 3.1416
	 z = 0.0
	 r = 0.0
         
         rospy.loginfo(a)
         pub_amp.publish(a)
         rospy.loginfo(t)
         pub_the.publish(t)
         rospy.loginfo(w)
         pub_vel.publish(w)
         rospy.loginfo(z)
         pub_red.publish(z)
         rospy.loginfo(r)
         pub_rad.publish(r)

         rate.sleep()
 
if __name__ == '__main__':
     try:
         talker()
     except rospy.ROSInterruptException:
         pass
