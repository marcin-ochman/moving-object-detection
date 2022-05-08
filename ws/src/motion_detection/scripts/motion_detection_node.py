#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import message_filters
from sklearn.metrics import confusion_matrix
import json
import disarray
import pandas as pd
import argparse


class FeatureMatcher:
    def __init__(self):
        self.feature_detector = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.max_matches_number = rospy.get_param('/max_matches_number', 20)

    def get_descriptors(self, image):
        keypoints, descriptors = self.feature_detector.detectAndCompute(image, None)
        return {"keypoints": keypoints, "descriptors": descriptors}

    def get_matches(self, prev_descriptors, current_descriptors):
        matches = ()
        if prev_descriptors is not None and current_descriptors is not None:
            matches = self.matcher.match(prev_descriptors, current_descriptors)
            matches = sorted(matches, key = lambda x:x.distance)
            matches = matches[:self.max_matches_number]
        return matches


class MotionDetectionNode:
    def __init__(self, threshold=0.7, gamma=0.4):
        rgb_sub = message_filters.Subscriber("rgb", Image)
        rgb_info_sub = message_filters.Subscriber("camera_info", CameraInfo)

        depth_sub = message_filters.Subscriber("depth", Image)
        mask_sub = message_filters.Subscriber("mask", Image)

        ts = message_filters.TimeSynchronizer([rgb_sub, rgb_info_sub, depth_sub, mask_sub], 10)
        ts.registerCallback(self.image_callback)

        self.cv_bridge = CvBridge()
        self.feature_matcher = FeatureMatcher()
        self.prev_image_points = {"keypoints": np.array([]), "descriptors": np.array([])}
        self.prev_img = None
        self.K = None
        self.position = np.eye(4)
        self.confusion_matrix = np.zeros((2, 2))

        self.threshold = threshold
        self.gamma = gamma

        self.alg = cv2.rgbd.Odometry_create('RgbdICPOdometry')

    def image_callback(self, image_msg, camera_info, depth_msg, mask_msg):

        self.K = np.array(camera_info.K).reshape(3,3)
        self.alg.setCameraMatrix(self.K)

        image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)
        mask = self.cv_bridge.imgmsg_to_cv2(mask_msg)

        current_points = self.feature_matcher.get_descriptors(image)
        matches = self.feature_matcher.get_matches(self.prev_image_points.get("descriptors"),
                                                   current_points.get("descriptors"))

        if len(matches) <= 5:
            self.prev_keypoint_weights = np.zeros(len(current_points['keypoints']))
            self.prev_image_points = current_points
            self.prev_img = image
            self.prev_depth_img = depth
            return

        keypoints = current_points['keypoints']
        prev_keypoints = self.prev_image_points["keypoints"]

        keypoints_2d = np.empty((len(matches), 2))
        prev_keypoints_2d = np.empty((len(matches), 2))
        weights = np.zeros(len(matches))
        for idx, match in enumerate(matches):
            keypoints_2d[idx, :] = keypoints[match.trainIdx].pt
            prev_keypoints_2d[idx, :] = prev_keypoints[match.queryIdx].pt
            weights[idx] = self.prev_keypoint_weights[match.queryIdx]


        if self.prev_img is not None:
            Rt = self.estimate_camera_movement(image, self.prev_img, depth, self.prev_depth_img)
            points3d = self.keypoints_to_3d(keypoints_2d, depth, self.K)
            prev_points3d = self.keypoints_to_3d(prev_keypoints_2d, self.prev_depth_img, self.K)

            new_weights = self.classify_points(Rt, points3d, prev_points3d, weights)

            keypoints_to_draw = []
            self.prev_keypoint_weights = np.zeros(len(keypoints))

            for idx, match in enumerate(matches):
                self.prev_keypoint_weights[match.trainIdx] = new_weights[idx]
                if new_weights[idx] > self.threshold:
                    keypoints_to_draw.append(keypoints[match.trainIdx])

            ref_class = self.get_ground_truth(keypoints_2d, mask)
            moving_classes = new_weights > self.threshold

            self.confusion_matrix += confusion_matrix(ref_class, moving_classes)


        self.prev_image_points = current_points
        self.prev_img = image
        self.prev_depth_img = depth

    def estimate_camera_movement(self, rgb, prev_rgb, depth, prev_depth):
        mask = np.logical_and(np.isfinite(depth), np.isfinite(prev_depth)).astype(np.uint8)
        dst_frame = cv2.rgbd.RgbdFrame_create(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY), depth)
        src_frame = cv2.rgbd.RgbdFrame_create( cv2.cvtColor(prev_rgb, cv2.COLOR_BGR2GRAY), prev_depth)
        success, Rt = self.alg.compute(src_frame.image, src_frame.depth, None,
                                       dst_frame.image, dst_frame.depth, None)

        return Rt if success else np.eye(4)

    def keypoints_to_3d(self, points2d, depth, K):
        cx = K[0, 2]
        cy = K[1, 2]
        fx = K[0, 0]
        fy = K[1, 1]

        img_coordinates = points2d.astype(np.int32)
        points3d = np.empty((img_coordinates.shape[0], 3))
        z = depth[img_coordinates[:, 1], img_coordinates[:, 0]].reshape(img_coordinates.shape[0], 1).reshape(img_coordinates.shape[0], 1)
        points3d[:, 0:2] = (points2d - [cx, cy])  * z / [fx, fy]
        points3d[:, 2] = z.flatten()

        return points3d

    def classify_points(self, camera_Rt, points, previous_points, weights):
        epsilon = 0.1
        update_weight = self.gamma
        R = camera_Rt[0:3, 0:3]
        t = camera_Rt[0:3, 3].reshape(3, 1)

        distance_classification = np.linalg.norm(np.matmul(R, previous_points.T) + t - points.T, axis=0) >= epsilon

        return distance_classification * update_weight + (1 - update_weight) * weights

    def get_ground_truth(self, points, mask):
        return mask[points[:, 1].astype(np.int32), points[:, 0].astype(np.int32)] > 0

    def dump_stats(self):
        out_path = '/moving_object_ws/data/results/result_theta_{}_threshold_{}.json'.format(self.gamma, self.threshold)
        pd_cm = pd.DataFrame(self.confusion_matrix, dtype=int)
        stats = {
            'gamma': self.gamma,
            'threshold': self.threshold,
            'confusion_matrix': self.confusion_matrix.flatten().tolist(),
            'f1': pd_cm.da.f1[1],
            'accuracy': pd_cm.da.accuracy[1],
            'recall': pd_cm.da.recall[1],
            'precision': pd_cm.da.precision[1]
        }

        with open(out_path, 'w') as f:
            json.dump(stats, f, indent=4)


if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=True)

    parser = argparse.ArgumentParser(description='RGBD with mask publisher')
    parser.add_argument('--threshold', type=float, help='Threshold for weights')
    parser.add_argument('--gamma', type=float, help='Update weight')

    argv = rospy.myargv()
    args = parser.parse_args(argv[1:])

    motion_detection_node = MotionDetectionNode(args.threshold, args.gamma)
    rospy.on_shutdown(motion_detection_node.dump_stats)
    rospy.spin()
