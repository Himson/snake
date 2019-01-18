import tty
import select
import sys
import termios

import rospy
import std_msgs.msg
settings = termios.tcgetattr(sys.stdin)
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

def key():
    pub = rospy.Publisher('joint', std_msgs.msg.String, queue_size=1)
    rospy.init_node('keyboard', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        keynow = getKey()
        if keynow == 'q':
            exit()
        print(keynow+'!')
        pub.publish(keynow)
        rate.sleep()
if __name__ == '__main__':
     try:		
         key()       
     except  rospy.ROSInterruptException:
         pass
