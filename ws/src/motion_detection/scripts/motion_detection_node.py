#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
import message_filters

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
        rgb_sub = message_filters.Subscriber("image", Image)
        rgb_info_sub = message_filters.Subscriber("camera_info", CameraInfo)

        depth_sub = message_filters.Subscriber("depth", Image)

        _sub = message_filters.Subscriber("image", Image)

        ts = message_filters.TimeSynchronizer([rgb_sub, rgb_info_sub, depth_sub], 10)
        ts.registerCallback(self.image_callback)

        self.cv_bridge = CvBridge()
        self.feature_matcher = FeatureMatcher()
        self.prev_image_points = {"keypoints": np.array([]), "descriptors": np.array([])}
        self.prev_img = None
        self.K = None
        self.position = np.eye(4)

        self.alg = cv2.rgbd.Odometry_create('RgbdICPOdometry')

    def image_callback(self, image_msg, camera_info, depth_msg):
        self.K = np.array(camera_info.K).reshape(3,3)
        self.alg.setCameraMatrix(self.K)

        image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        current_points = self.feature_matcher.get_descriptors(image)
        matches = self.feature_matcher.get_matches(self.prev_image_points.get("descriptors"),
                                                   current_points.get("descriptors"))

        if self.prev_img is not None:
            cv2.imshow("Feature points", cv2.drawMatches( self.prev_img, self.prev_image_points["keypoints"],
                                                          image, current_points['keypoints'], matches, None))
            cv2.waitKey(1)

            # self.estimate_camera_movement(current_points['keypoints'],
            #                               self.prev_image_points['keypoints'],
            #                               matches, depth, self.prev_depth_img, self.K)
            Rt = self.estimate_camera_movement(image, self.prev_img, depth, self.prev_depth_img)

            self.position = np.matmul(self.position, Rt)

            rospy.logwarn('\n --------- \n t: {}\n --------- \n'.format(self.position[:, 3]))

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


    # def estimate_camera_movement(self, keypoints, prev_keypoints, matches, depth, prev_depth, K):
    #     keypoints_pts = np.empty((len(matches), 2))
    #     prev_keypoints_pts = np.empty((len(matches), 2))

    #     if len(matches) <= 5:
    #         return

    #     for idx, match in enumerate(matches):
    #         keypoints_pts[idx, :] = keypoints[match.trainIdx].pt
    #         prev_keypoints_pts[idx, :] = prev_keypoints[match.queryIdx].pt

    #     E, _ = cv2.findEssentialMat(prev_keypoints_pts, keypoints_pts,  self.K);
    #     retval, R, t, _ = cv2.recoverPose(E, prev_keypoints_pts, keypoints_pts, self.K)

    #     if retval / keypoints_pts.shape[0] < 0.4:
    #         rospy.logerr("Error estimating R,t: {}".format(retval / keypoints_pts.shape[0] * 100))
    #         return

    #     rospy.logerr("It's ok")

    #     C = np.array([[*R[0, :], t[0, 0]],
    #                   [*R[1, :], t[1, 0]],
    #                   [*R[2, :], t[2, 0]],
    #                    [0, 0, 0, 1]])

    #     scale = self.estimate_scale(keypoints_pts[0, :], prev_keypoints_pts[0, :], depth, prev_depth, K, R)

    #     self.position += scale * np.matmul(self.rotation, t)
    #     self.rotation = np.matmul(R, self.rotation)

    def estimate_scale(self, point, prev_point, depth, prev_depth,K, R):
        cx = K[0, 2]
        cy = K[1, 2]
        fx = K[0, 0]
        fy = K[1, 1]

        point_3d = np.array([(point[0] - cx) * depth[int(point[1]), int(point[0])]/fx,
                             (point[1] - cy) * depth[int(point[1]), int(point[0])]/fy,
                             depth[int(point[1]), int(point[0])]]).T

        prev_point_3d = np.array([(prev_point[0] - cx) * prev_depth[int(prev_point[1]), int(prev_point[0])]/fx,
                                  (prev_point[1] - cy) * prev_depth[int(prev_point[1]), int(prev_point[0])]/fy,
                                  prev_depth[int(prev_point[1]), int(prev_point[0])]]).T

        rospy.logwarn('\n----------- \n\n Pn:\n {0} \n Pn-1:\n {1}\n R:\n {4} \n RXn-1: {3} \n scale: {2}\n ------------\n\n'.format(point_3d,
                                                                                                      prev_point_3d,
                                                                                                                        np.linalg.norm(point_3d - np.matmul(R, prev_point_3d)),
                                                                                                                            (point_3d - np.matmul(R, prev_point_3d)).flatten(),
                                                                                                                            R
                                                                                                      ))

        return np.linalg.norm(point_3d - np.matmul(R, prev_point_3d))



if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=True)
    motion_detection_node = MotionDetectionNode()
    rospy.spin()
