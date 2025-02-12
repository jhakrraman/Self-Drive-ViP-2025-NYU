#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")

def my_node():
    rospy.init_node('my_node', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    pub = rospy.Publisher('response', String, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish("Hello from my_node 3!")
        rate.sleep()

if __name__ == '__main__':
    try:
        my_node()
    except rospy.ROSInterruptException:
        pass
