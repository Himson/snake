import rospy
from dvs_msgs.msg import EventArray
import sensor_msgs.msg._Image as img
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def display_events(data, args):
    pub = args[0]
    rospy.loginfo(rospy.get_caller_id() + "I heard.")
    event_msg = data
    if event_msg is None:
        return
    rendered_img = np.zeros((128, 128, 3), dtype=np.uint8)
    for event in event_msg.events:
        rendered_img[event.y][event.x] = (event.polarity * 255, 255, 0)
    msg_frame = CvBridge().cv2_to_imgmsg(rendered_img, 'rgb8')
    rospy.loginfo("I sent.")
    pub.publish(msg_frame)

def listener():
    dvs = rospy.init_node('dvs_camera', anonymous=True)
    pub_render= rospy.Publisher('/dvs_rendered_full', img.Image, queue_size=10)
    sub_event = rospy.Subscriber('head/dvs128/events', EventArray, display_events, (pub_render, None))
    rospy.spin()

if __name__ == '__main__':
    try:
	listener()
    except rospy.ROSInterruptException:
	pass
