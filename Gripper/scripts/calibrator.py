#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Bool

def talker():
    pub = rospy.Publisher('gripper_state', Bool, queue_size=10)
    rospy.init_node('Gripper_state', anonymous=True)
    rate = rospy.Rate(0.1) # 1hz
    msg = Bool()
    state = False
    while not rospy.is_shutdown():
        if state == False:
            state = True
        else:
            state = False 
        msg.data = state
        rospy.loginfo(state)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
