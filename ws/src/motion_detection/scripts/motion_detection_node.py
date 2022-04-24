#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np


class FeatureMatcher:
    def __init__(self):
        self.feature_detector = cv2.ORB_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.max_matches_number = rospy.get_param('max_matches_number', 10)

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
    def __init__(self):
        self.sub = rospy.Subscriber("image", Image, self.image_callback)
        self.cv_bridge = CvBridge()
        self.feature_matcher = FeatureMatcher()
        self.prev_image_points = {"keypoints": np.array([]), "descriptors": np.array([])}

    def image_callback(self, image_msg):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)
        current_points = self.feature_matcher.get_descriptors(image)
        matches = self.feature_matcher.get_matches(self.prev_image_points.get("descriptors"),
                                                   current_points.get("descriptors"))
        self.prev_image_points = current_points


if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=True)
    motion_detection_node = MotionDetectionNode()
    rospy.spin()
