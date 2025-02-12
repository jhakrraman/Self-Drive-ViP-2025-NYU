#!/usr/bin/env python3

import rospy
import subprocess
import signal
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

class BagPlayerNode:
    def __init__(self):
        rospy.init_node('bag_player_node')
        
        # Get parameters
        self.bag_path = rospy.get_param('~bag_path', 'name_of_bag.bag')
        self.image_topic = rospy.get_param('~image_topic', '/camera/image')
        
        # Initialize variables
        self.target_id = None
        self.image_count = 0
        self.bag_process = None
        self.image_sub = None

        # It tends to overshoot the target, so we need to correct for it
        self.correction_factor = 4 # making it stops four images prior

        # Subscribe to match_goal_id topic
        rospy.Subscriber('/match_goal_id', Int32, self.id_callback)
        rospy.loginfo("Initialized. Waiting for target ID on /match_goal_id...")

    def id_callback(self, msg):
        if self.target_id is not None:
            rospy.logwarn("Already processing a bag. Ignoring new ID.")
            return

        self.target_id = msg.data
        rospy.loginfo(f"Received target ID: {self.target_id}. Starting rosbag play...")
        
        # Start rosbag play
        self.bag_process = subprocess.Popen(['rosbag', 'play', self.bag_path])
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"Monitoring images on {self.image_topic}")

    def image_callback(self, msg):
        self.image_count += 1
        rospy.logdebug(f"Image count: {self.image_count}")
        print(self.image_count)
        if self.image_count >= self.target_id - self.correction_factor:
            rospy.loginfo(f"Reached target image count ({self.target_id}). Stopping rosbag...")
            
            # Stop rosbag process
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            
            # Cleanup and shutdown
            self.image_sub.unregister()
            rospy.signal_shutdown("Target image count reached")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BagPlayerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass