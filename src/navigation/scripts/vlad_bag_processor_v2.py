#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import pickle
import rosbag
from sklearn.cluster import KMeans
from sklearn.neighbors import BallTree
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, String, Float32, Int32, Time

class VLADBagProcessor:
    def __init__(self):
        rospy.init_node('vlad_bag_processor')

        # Initialize publishers
        self.status = rospy.Publisher('/match_status', Bool, queue_size=10)
        self.timestamp_pub = rospy.Publisher('/goal_timestamp', Time, queue_size=10)
        self.goal_id = rospy.Publisher('/match_goal_id', Int32, queue_size=10)
        
        # Load parameters
        self.codebook_path = rospy.get_param('~codebook_path', 'codebook.pkl')
        target_image_path = rospy.get_param('~target_image_path', 'target.jpg')
        self.bag_path = rospy.get_param('~bag_file_path', 'images.bag')
        self.num_clusters = rospy.get_param('~num_clusters', 64)

        # Initialize SIFT and CV bridge
        self.sift = cv2.SIFT_create()
        self.bridge = CvBridge()

        # Load or generate codebook
        if not self.load_codebook():
            rospy.loginfo("No codebook found. Generating new codebook...")
            self.generate_codebook()
            self.save_codebook()

        # Build VLAD database and search tree
        self.database = []
        self.timestamps = []
        self.build_vlad_database()
        self.tree = BallTree(self.database, leaf_size=40)

        # Process target image and find goal index
        self.target_image = cv2.imread(target_image_path)
        if self.target_image is None:
            raise FileNotFoundError(f"Target image not found at {target_image_path}")
        
        target_vlad = self.compute_vlad(self.target_image)
        if target_vlad is None:
            raise RuntimeError("Failed to compute VLAD for target image")
        
        _, self.goal_index = self.tree.query(target_vlad.reshape(1, -1), k=1)
        self.goal_index = self.goal_index[0][0]
        self.goal_timestamp = self.timestamps[self.goal_index]
        
        rospy.loginfo(f"Goal index: {self.goal_index}")

        self.status.publish(True)
        self.goal_id.publish(self.goal_index)
        self.timestamp_pub.publish(Time(self.goal_timestamp))


    def load_codebook(self):
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
        with open(self.codebook_path, 'wb') as f:
            pickle.dump(self.codebook, f)
        rospy.loginfo(f"Codebook saved to {self.codebook_path}")

    def compute_sift_features(self):
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
        return np.array(sift_descriptors)

    def generate_codebook(self):
        sift_descriptors = self.compute_sift_features()
        if len(sift_descriptors) < self.num_clusters:
            raise ValueError("Insufficient SIFT descriptors for codebook generation")
        self.codebook = KMeans(n_clusters=self.num_clusters, random_state=0).fit(sift_descriptors)

    def compute_vlad(self, img):
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

    def build_vlad_database(self):
        try:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for _, msg, t in bag.read_messages(topics=['/camera/image']):
                    self.timestamps.append(t)

                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        vlad = self.compute_vlad(cv_image)
                        self.database.append(vlad if vlad is not None else np.zeros(self.num_clusters*128))
                    except CvBridgeError as e:
                        rospy.logwarn(f"Skipping image: {e}")
                        self.database.append(np.zeros(self.num_clusters*128))
        except Exception as e:
            rospy.logerr(f"Error building VLAD database: {e}")
            raise

if __name__ == '__main__':
    try:
        VLADBagProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass