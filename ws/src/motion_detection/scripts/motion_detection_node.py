#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np

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
    def __init__(self):
        self.sub = rospy.Subscriber("image", Image, self.image_callback)
        self.sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback)
        self.cv_bridge = CvBridge()
        self.feature_matcher = FeatureMatcher()
        self.prev_image_points = {"keypoints": np.array([]), "descriptors": np.array([])}
        self.prev_img = None
        self.K = None
        self.position = np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0],
                                  [0, 0, 0, 1]])

    def camera_info_callback(self, camera_info_msg):
        if self.K is None:
            self.K = np.array(camera_info_msg.K).reshape(3,3)

    def image_callback(self, image_msg):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)
        current_points = self.feature_matcher.get_descriptors(image)
        matches = self.feature_matcher.get_matches(self.prev_image_points.get("descriptors"),
                                                   current_points.get("descriptors"))

        # cv2.imshow("Feature points", cv2.drawKeypoints(image, current_points['keypoints'], None))
        if self.prev_img is not None:
            cv2.imshow("Feature points", cv2.drawMatches(image, current_points['keypoints'], self.prev_img,
                                                         self.prev_image_points["keypoints"], matches, None))
            cv2.waitKey(1)


        self.estimate_essential_matrix(current_points['keypoints'],
                                       self.prev_image_points['keypoints'],
                                       matches)

        self.prev_image_points = current_points
        self.prev_img = image

    def estimate_essential_matrix(self, keypoints, prev_keypoints, matches):
        keypoints_pts = np.empty((len(matches), 2))
        prev_keypoints_pts = np.empty((len(matches), 2))

        if len(matches) <= 5:
            return

        for idx, match in enumerate(matches):
            keypoints_pts[idx, :] = keypoints[match.trainIdx].pt
            prev_keypoints_pts[idx, :] = prev_keypoints[match.queryIdx].pt

        E, _ = cv2.findEssentialMat(keypoints_pts, prev_keypoints_pts, self.K);
        _, R, t, _ = cv2.recoverPose(E, keypoints_pts, prev_keypoints_pts, self.K)

        C = np.array([[*R[0, :], t[0, 0]],
                      [*R[1, :], t[1, 0]],
                      [*R[2, :], t[2, 0]],
                       [0, 0, 0, 1]])

        self.position = np.matmul(self.position, C)



if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=True)
    motion_detection_node = MotionDetectionNode()
    rospy.spin()
