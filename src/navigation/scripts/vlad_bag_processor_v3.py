#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
import pickle
import rosbag
import torch
import torchvision.models as models
import torchvision.transforms as transforms
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool, Int32, Time
from sklearn.cluster import KMeans
from sklearn.neighbors import BallTree

class VLADBagProcessorV3:
    def __init__(self):
        rospy.init_node('vlad_bag_processor_v3')

        # Publishers for match status, goal index, and timestamp.
        self.match_status_pub = rospy.Publisher('/match_status', Bool, queue_size=10)
        self.goal_id_pub = rospy.Publisher('/match_goal_id', Int32, queue_size=10)
        self.goal_timestamp_pub = rospy.Publisher('/goal_timestamp', Time, queue_size=10)

        # Load parameters.
        self.codebook_path = rospy.get_param('~codebook_path', 'codebook_cnn.pkl')
        self.target_image_path = rospy.get_param('~target_image_path', 'target.jpg')
        self.bag_path = rospy.get_param('~bag_file_path', 'images.bag')
        self.num_clusters = rospy.get_param('~num_clusters', 64)
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.4)

        # Initialize CV Bridge.
        self.bridge = CvBridge()

        # Initialize pre-trained ResNet50 for CNN feature extraction (remove avgpool & fc layers).
        resnet = models.resnet50(pretrained=True)
        modules = list(resnet.children())[:-2]
        self.cnn_model = torch.nn.Sequential(*modules)
        self.cnn_model.eval()

        # Define image preprocessing transform.
        self.preprocess = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406],
                                 std=[0.229, 0.224, 0.225]),
        ])

        # Load or generate the codebook.
        if not self.load_codebook():
            rospy.loginfo("Codebook not found. Generating new codebook using CNN features...")
            self.generate_codebook()
            self.save_codebook()

        # Build VLAD database from bag images.
        self.database = []      # VLAD descriptors.
        self.timestamps = []    # Corresponding timestamps.
        self.build_vlad_database()

        # Build BallTree for fast nearest-neighbor search.
        self.tree = BallTree(np.array(self.database), leaf_size=40)

        # Process the target image and perform feature matching.
        self.process_target_image()

    def load_codebook(self):
        if os.path.exists(self.codebook_path):
            try:
                with open(self.codebook_path, 'rb') as f:
                    self.codebook = pickle.load(f)
                rospy.loginfo("Codebook loaded successfully.")
                return True
            except Exception as e:
                rospy.logwarn("Failed to load codebook: %s", str(e))
        return False

    def save_codebook(self):
        with open(self.codebook_path, 'wb') as f:
            pickle.dump(self.codebook, f)
        rospy.loginfo("Codebook saved to %s", self.codebook_path)

    def compute_cnn_descriptors(self, cv_image):
        # Convert BGR image to RGB and then to a PIL image.
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = Image.fromarray(rgb_image)
        input_tensor = self.preprocess(pil_image)
        input_batch = input_tensor.unsqueeze(0)  # Add batch dimension.

        with torch.no_grad():
            features = self.cnn_model(input_batch)  # Shape: (1, C, H, W)
        features = features.squeeze(0)  # Shape: (C, H, W)
        # Reshape: each spatial location becomes a descriptor (H*W, C)
        descriptors = features.permute(1, 2, 0).reshape(-1, features.shape[0])
        return descriptors.cpu().numpy()

    def generate_codebook(self):
        all_descriptors = []
        try:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for _, msg, _ in bag.read_messages(topics=['/camera/image']):
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        descriptors = self.compute_cnn_descriptors(cv_image)
                        if descriptors is not None and len(descriptors) > 0:
                            all_descriptors.extend(descriptors)
                    except CvBridgeError as e:
                        rospy.logwarn("CvBridge error: %s", str(e))
        except Exception as e:
            rospy.logerr("Error reading bag file: %s", str(e))
            raise
        all_descriptors = np.array(all_descriptors)
        if len(all_descriptors) < self.num_clusters:
            raise ValueError("Insufficient CNN descriptors for codebook generation")
        # Generate codebook using KMeans.
        self.codebook = KMeans(n_clusters=self.num_clusters, random_state=0).fit(all_descriptors)

    def compute_vlad(self, cv_image):
        descriptors = self.compute_cnn_descriptors(cv_image)
        if descriptors is None or len(descriptors) == 0:
            return None
        k = self.codebook.n_clusters
        centroids = self.codebook.cluster_centers_
        pred_labels = self.codebook.predict(descriptors)
        vlad = np.zeros((k, descriptors.shape[1]))
        for i in range(k):
            if np.sum(pred_labels == i) > 0:
                vlad[i] = np.sum(descriptors[pred_labels == i] - centroids[i], axis=0)
        vlad = vlad.flatten()
        # Apply power normalization.
        vlad = np.sign(vlad) * np.sqrt(np.abs(vlad))
        norm = np.linalg.norm(vlad)
        if norm > 0:
            vlad /= norm
        return vlad

    def build_vlad_database(self):
        try:
            with rosbag.Bag(self.bag_path, 'r') as bag:
                for _, msg, t in bag.read_messages(topics=['/camera/image']):
                    self.timestamps.append(t)
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                        vlad = self.compute_vlad(cv_image)
                        if vlad is None:
                            # Fallback: create a zero vector if VLAD computation fails.
                            vlad = np.zeros(self.num_clusters * 2048)
                        self.database.append(vlad)
                    except CvBridgeError as e:
                        rospy.logwarn("Error converting image: %s", str(e))
                        vlad = np.zeros(self.num_clusters * 2048)
                        self.database.append(vlad)
        except Exception as e:
            rospy.logerr("Error building VLAD database: %s", str(e))
            raise

    def process_target_image(self):
        target_image = cv2.imread(self.target_image_path)
        if target_image is None:
            rospy.logerr("Target image not found at %s", self.target_image_path)
            raise FileNotFoundError("Target image not found")
        target_vlad = self.compute_vlad(target_image)
        if target_vlad is None:
            rospy.logerr("Failed to compute VLAD for target image")
            raise RuntimeError("Failed to compute VLAD for target image")
        # Query the BallTree for the nearest neighbor.
        dist, idx = self.tree.query(target_vlad.reshape(1, -1), k=1)
        goal_index = idx[0][0]
        goal_timestamp = self.timestamps[goal_index]
        rospy.loginfo("Target match found at index %d, timestamp %s", goal_index, str(goal_timestamp))
        # Publish the match result.
        self.match_status_pub.publish(True)
        self.goal_id_pub.publish(goal_index)
        self.goal_timestamp_pub.publish(Time(goal_timestamp))

if __name__ == '__main__':
    try:
        processor = VLADBagProcessorV3()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
