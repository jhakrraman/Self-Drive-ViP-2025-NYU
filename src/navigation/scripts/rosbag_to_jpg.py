#!/usr/bin/env python3

import os
import rosbag
from cv_bridge import CvBridge
import cv2

def save_images_from_bag(bag_path, output_dir, image_topic):
    """
    Save images from ROSBAG to JPEG files
    :param bag_path: Path to input ROSBAG file
    :param output_dir: Directory to save images
    :param image_topic: ROS topic containing images
    """
    # Create output directory if needed
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    bridge = CvBridge()
    count = 0
    error_count = 0

    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            # Get total messages for progress display
            total = bag.get_message_count(topic_filters=[image_topic])
            print(f"Processing {total} images from {bag_path}")

            for topic, msg, t in bag.read_messages(topics=[image_topic]):
                try:
                    # Convert ROS image message to OpenCV
                    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                    
                    # Save image with sequential numbering
                    filename = os.path.join(output_dir, f"{count:08d}.jpg")
                    cv2.imwrite(filename, cv_image)
                    count += 1
                    
                    # Print progress every 100 images
                    if count % 100 == 0:
                        print(f"Saved {count}/{total} images")

                except Exception as e:
                    print(f"Error processing image {count}: {str(e)}")
                    error_count += 1

    except Exception as e:
        print(f"Error opening bag file: {str(e)}")
        return

    print(f"\nDone! Success: {count}, Errors: {error_count}")
    print(f"Images saved to: {os.path.abspath(output_dir)}")

if __name__ == "__main__":
    # Edit these values as needed
    BAG_FILE = "/home/linux/catkin_s25/bags/target_image_2025-02-10-21-13-09.bag"  # Update this path
    OUTPUT_DIR = "/home/linux/catkin_s25/image_data/target image"       # Update if needed
    IMAGE_TOPIC = "/camera/image"         # Update if your topic is different
    
    save_images_from_bag(BAG_FILE, OUTPUT_DIR, IMAGE_TOPIC)