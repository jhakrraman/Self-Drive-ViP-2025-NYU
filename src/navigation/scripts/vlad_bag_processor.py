#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import pickle
import rosbag
from sklearn.cluster import KMeans
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

class VLADBagProcessor:
    def __init__(self):
        rospy.init_node('vlad_bag_processor')

        # Load parameters
        self.codebook_path = rospy.get_param('~codebook_path', 'codebook.pkl')
        target_image_path = rospy.get_param('~target_image_path', 'target.jpg')
        self.bag_path = rospy.get_param('~bag_file_path', 'images.bag')
        self.num_clusters = rospy.get_param('~num_clusters', 64)
        self.threshold = rospy.get_param('~distance_threshold', 0.4)

        # Initialize SIFT and CV bridge
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.bridge = CvBridge()
        self.status_pub = rospy.Publisher('/match_status', Bool, queue_size=10)

        # Load or generate codebook
        if not self.load_codebook():
            rospy.loginfo("No codebook found. Generating new codebook...")
            self.generate_codebook()
            self.save_codebook()

        # Load and process target image
        self.target_image = cv2.imread(target_image_path)
        if self.target_image is None:
            rospy.logerr(f"Target image not found at {target_image_path}")
            raise FileNotFoundError(f"Target image not found at {target_image_path}")
        
        self.target_vlad = self.compute_vlad(self.target_image)
        if self.target_vlad is None:
            rospy.logerr("Failed to compute VLAD for target image.")
            raise RuntimeError("Invalid target image - no SIFT features detected.")

        # Process bag file
        self.process_bag()

    def load_codebook(self):
        """Attempt to load existing codebook"""
        if os.path.exists(self.codebook_path):
            try:
                with open(self.codebook_path, 'rb') as f:
                    self.codebook = pickle.load(f)
                rospy.loginfo("Codebook loaded successfully.")
                return True
            except Exception as e:
                rospy.logwarn(f"Failed to load codebook: {e}")
        return False

    def save_codebook(self):
        """Save generated codebook to file"""
        with open(self.codebook_path, 'wb') as f:
            pickle.dump(self.codebook, f)
        rospy.loginfo(f"Codebook saved to {self.codebook_path}")

    def compute_sift_features(self):
        """Extract SIFT features from all images in ROSBAG"""
        sift_descriptors = []
        try:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for _, msg, _ in bag.read_messages(topics=['/camera/image']):
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        _, des = self.sift.detectAndCompute(cv_image, None)
                        if des is not None:
                            sift_descriptors.extend(des)
                    except CvBridgeError as e:
                        rospy.logwarn(f"Skipping image: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing bag file: {e}")
            raise

        rospy.loginfo(f"Collected {len(sift_descriptors)} SIFT descriptors")
        return np.array(sift_descriptors)

    def generate_codebook(self):
        """Generate new codebook using KMeans clustering"""
        sift_descriptors = self.compute_sift_features()
        
        if len(sift_descriptors) == 0:
            raise ValueError("No SIFT descriptors found in dataset")
        
        if len(sift_descriptors) < self.num_clusters:
            raise ValueError(f"Not enough descriptors ({len(sift_descriptors)}) for {self.num_clusters} clusters")

        rospy.loginfo(f"Generating codebook with {self.num_clusters} clusters...")
        kmeans = KMeans(n_clusters=self.num_clusters, random_state=0)
        kmeans.fit(sift_descriptors)
        self.codebook = kmeans
        rospy.loginfo("Codebook generation complete")

    def compute_vlad(self, img):
        """Compute VLAD descriptor for an image"""
        _, des = self.sift.detectAndCompute(img, None)
        if des is None:
            return None

        k = self.codebook.n_clusters
        centroids = self.codebook.cluster_centers_
        pred_labels = self.codebook.predict(des)

        vlad = np.zeros((k, des.shape[1]))
        for i in range(k):
            if np.sum(pred_labels == i) > 0:
                vlad[i] = np.sum(des[pred_labels == i] - centroids[i], axis=0)

        vlad = vlad.flatten()
        vlad = np.sign(vlad) * np.sqrt(np.abs(vlad))
        vlad /= np.linalg.norm(vlad)
        return vlad

    def process_bag(self):
        """Process all images in the ROSBAG file"""
        try:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for topic, msg, t in bag.read_messages(topics=['/camera/image']):
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                    except CvBridgeError as e:
                        rospy.logwarn(f"Image conversion failed: {e}")
                        continue

                    current_vlad = self.compute_vlad(cv_image)
                    if current_vlad is None:
                        self.status_pub.publish(Bool(False))
                        continue

                    distance = np.linalg.norm(current_vlad - self.target_vlad)
                    is_match = distance < self.threshold
                    self.status_pub.publish(Bool(is_match))
                    rospy.loginfo(f"Processed image at {t.to_sec()}: Match={is_match}, Distance={distance:.4f}")

        except Exception as e:
            rospy.logerr(f"Error processing bag file: {e}")
            raise

if __name__ == '__main__':
    try:
        processor = VLADBagProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass