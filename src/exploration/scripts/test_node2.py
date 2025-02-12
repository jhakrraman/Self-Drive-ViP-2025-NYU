#!/usr/bin/env python3

import rospy

def main():
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("My ROS Node has started!")
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Node 2 is running...")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass