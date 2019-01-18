import rospy
import std_msgs.msg
import numpy as np
import StringIO
module_num= 17
joint_num = 16
pub_joints= []
global mod
mod = False
global flag
angle = []
flag = True
amp		    = 10.0*np.pi/180.0
phase_diff 	= 0.5*np.pi
omega 		= 0.9*np.pi
now = None
t = 0.0
global out
out = None
global Step
Step = 1
for i in range(1,joint_num+1):
	topic = '/wl_snake/joint_%d/cmd_pos'%i
	pub = rospy.Publisher(topic, std_msgs.msg.Float64, queue_size=10)
	pub_joints.append(pub)
	#angle.append(0.0)
	angle.append(3.0/180.0*np.pi*np.cos(0.5*np.pi*i))
	# if i%2 !=0:
    #         angle.append(70.0)
	# else:
    #         angle.append(0.0)
def anglego(m,n):
    diff = n-angle[m]
    while np.abs(diff) > 0.01:
        angle[m] += diff/10
        diff = n - angle[m]
        pub_joints[m].publish(angle[m])
        rospy.sleep(0.01)
    global Step
    print("step %d"%Step)
    Step += 1
    rospy.sleep(0.3)
def cb(key):
    sio = StringIO.StringIO()
    sio.write(key)
    global out
    out = sio.getvalue()[7]
    global flag
    if out == '/':
        global mod
        mod = not mod
        print("the mode now is %r" %mod)
    if mod == False:
        if out == '-':
            flag = not flag
            print("flag is now %r"%flag)
        else:
            j = ord(out) - ord('a')
            if j > 15 or j < 0:
                print("You are in the mod False")
            else:    
                if flag == False:
                    angle[j] -= 0.1
                    print("angle %d now is %f"%(j,angle[j]))
                else:
                    angle[j] += 0.1
                    print("angle %d now is %f"%(j,angle[j]))
    if out == '1':
        # for j in range(10):
        #     for i in range(joint_num):
        #         if i == 3:
        #             angle[3] += (90.0/180.0*np.pi - angle[3])/2
        #         elif i == 13:
        #             angle[13] += (-90.0/180.0*np.pi - angle[13])/2
        #         else:
        #             angle[i]*=0.7
        #         pub_joints[i].publish(angle[i])
        # for i in range(joint_num):
        #     angle[i] = 3.0/180.0*np.pi*np.cos(0.5*np.pi*i)
        #     pub_joints[i].publish(angle[i])
        # # current_amp = amp
        # angle[10] = -1.7
        # pub_joints[10].publish(angle[10])
        anglego(10,-1.7)
        anglego(12,1.7)
        anglego(1,1.7)
        anglego(3,-1.7)
        anglego(1,0)
        anglego(5,1.8)
        anglego(3,0)
        anglego(7,-1.8)
        anglego(5,0)
        anglego(4,-1.7)
        anglego(5,-1.8)
        anglego(9,-1.9)
        anglego(7,0)
        anglego(5,0)
        anglego(7,1.0)
        anglego(13,1.8)
        anglego(6,-1.6)
        anglego(7,0.0)
        anglego(4,-0.6)
        anglego(6,-0.0)
        anglego(4,0.0)
        anglego(10,1.6)
        anglego(3,0.3)
        anglego(11,1.5)
        anglego(9,0.0)
        anglego(11,0.0)
        out = 'r'
        # anglego(11,1.4)
        # anglego(9,0.0)
        # anglego(12,0.0)
        # anglego(11,0.0)
        # anglego(13,0.0)
        print("Done!")
        # while current_amp < 40.0:
        #     current_amp += 0.05
        #     for i in range(joint_num):
        #         angle[i] = -current_amp/180.0*np.pi*np.sin(np.pi+0.5*np.pi*i)
        #         pub_joints[i].publish(angle[i])
    if out == '2':
        while angle[4] > -1.5:
            angle[4] -= 0.01
            pub_joints[4].publish(angle[4])
        print("step 2 finished")
        rospy.sleep(1)
        for i in [3,2,1]:
            for it in range(10):
                angle[i] *= 0.7
                pub_joints[i].publish(angle[i])
        print("step 3 finished")
        rospy.sleep(1)
        while angle[6] < 1.5:
            angle[6] += 0.1
            pub_joints[6].publish(angle[6])
        print("step 4 finished")
        rospy.sleep(1)
        angle[5] = -1.5
        pub_joints[5].publish(angle[5])
        print("step 5 finished")
        rospy.sleep(1)
        angle[3] = -1.2
        pub_joints[3].publish(angle[3])
        rospy.sleep(1)
        angle[4] = 0.0
        pub_joints[4].publish(angle[4])
        rospy.sleep(1)
        angle[3] = 0.0
        pub_joints[3].publish(angle[3])
        print("step 6 finished")
        rospy.sleep(1)
        angle[8] = 1.7
        pub_joints[8].publish(angle[8])
        print("step 7 finished")     
        rospy.sleep(1) 
        angle[6] = 0.0
        pub_joints[8].publish(angle[8])
        print("step 8 finished")     
        rospy.sleep(1)   
        rospy.sleep(1)  
        angle[5] = 0.0
        pub_joints[5].publish(angle[5])
        print("step 9 finished")     
        rospy.sleep(1)
        angle[6] = 1.0
        pub_joints[6].publish(angle[7])
        print("step 10 finished")  
        angle[7] = 0.0
        pub_joints[7].publish(angle[7])
        print("step 11 finished")  
        angle[6] = 0.2
        pub_joints[6].publish(angle[6])
        print("step 12 finished") 
        angle[9] = 0.3
        pub_joints[9].publish(angle[9])
        print("step 13 finished")  
        rospy.sleep(1)
        angle[9] = -1.7
        pub_joints[9].publish(angle[9])
        print("step 14 finished")  
        rospy.sleep(1)
        angle[8] = 0.0
        pub_joints[9].publish(angle[8])
        print("step 15 finished")  
        rospy.sleep(1)
        angle[10] = 1.8
        pub_joints[10].publish(angle[10])
        print("step 16 finished")  
        rospy.sleep(1) 
        angle[9] = 0.0
        pub_joints[9].publish(angle[9])
        print("step 17 finished")  
        rospy.sleep(1)
        angle[11] = -1.7
        pub_joints[11].publish(angle[11])
        print("step 18 finished")  
        rospy.sleep(1) 
        angle[10] = 0.0
        pub_joints[10].publish(angle[10])
        print("step 19 finished")  
        rospy.sleep(1) 
        angle[12] = 1.8
        pub_joints[12].publish(angle[12])
        print("step 20 finished")  
        rospy.sleep(1)  
        angle[11] = 0.0
        pub_joints[11].publish(angle[11])
        print("step 21 finished")  
        rospy.sleep(1)   
        angle[13] = -1.8
        pub_joints[13].publish(angle[13])
        print("step 22 finished")  
        rospy.sleep(1)
        angle[12] = 0.0
        pub_joints[12].publish(angle[12])
        print("step 21 finished")  
        rospy.sleep(1)  
        # angle[10] = -1.5
        # pub_joints[10].publish(angle[10])
        # print("step 7 finished")
        # rospy.sleep(1)
        # angle[7] = -1.3
        # pub_joints[10].publish(angle[7])
        # print("step 8 finished")
    

            
def gaitwithkeyboard():
    rospy.init_node('gaitwithkeyboard', anonymous=True)
    rospy.Subscriber('/joint', std_msgs.msg.String,cb)
    rate = rospy.Rate(100)
    global out
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        t = rospy.Time(now.secs, now.nsecs).to_sec()
        if out == '0':
            for i in range(joint_num):
                angle[i] = 0.0 
        if out == 'r':
            for i in range(joint_num):
                angle[i] = -amp*np.sin(omega*t+phase_diff*i)
        for i in range(joint_num):
            pub_joints[i].publish(angle[i])

        rate.sleep()
    rospy.spin() 


if __name__ == '__main__':
    try:
        gaitwithkeyboard()
    except rospy.ROSInterruptException:
        pass