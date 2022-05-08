#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import cv2 as cv
import numpy as np
import message_filters
import argparse
from os.path import splitext, isfile, join
from os import listdir
import sys


def is_png(png_path):
    return isfile(png_path) and splitext(png_path)[-1].lower() == '.png'


def is_bmp(png_path):
    return isfile(png_path) and splitext(png_path)[-1].lower() == '.bmp'


def import_images(base_dir):
    return sorted([join(base_dir, f) for f in listdir(base_dir) if is_png(join(base_dir, f)) or is_bmp(join(base_dir, f))])


class RgbdPublisher:
    def __init__(self, rgb_path, depth_path, mask_path):
        self.rgb_paths = import_images(rgb_path)
        self.depth_paths = import_images(depth_path)
        self.mask_paths = import_images(mask_path)

        self.index = 0

        assert len(self.rgb_paths) == len(self.depth_paths)
        assert len(self.rgb_paths) == len(self.mask_paths)

        self.rgb_publisher = rospy.Publisher('rgb', Image, queue_size=5)
        self.depth_publisher = rospy.Publisher('depth', Image, queue_size=5)
        self.mask_publisher = rospy.Publisher('mask', Image, queue_size=5)
        self.camera_info_publisher = rospy.Publisher('camera_info', CameraInfo, queue_size=5)
        self.timer = rospy.Timer(rospy.Duration(0.2), self._timer_callback)
        self.bridge = CvBridge()

    def publish_camera_info(self):
        camera_info = CameraInfo()
        camera_info.K = [554.254691191187, 0.0, 320.5, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 1.0]
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.height = 480
        camera_info.width = 640
        camera_info.P = [554.254691191187, 0.0, 320.5, -0.0, 0.0, 554.254691191187, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        camera_info.distortion_model = "plumb_bob"

        self.camera_info_publisher.publish(camera_info)

    def _timer_callback(self, timer):
        self.publish_next()

    def publish_next(self):
        if self.index >= len(self.rgb_paths):
            return

        rgb = cv.imread(self.rgb_paths[self.index])
        depth = (cv.imread(self.depth_paths[self.index], cv.IMREAD_ANYDEPTH) / 1000.0).astype(np.float32)
        mask = cv.imread(self.mask_paths[self.index], cv.IMREAD_GRAYSCALE)

        rospy.loginfo('\n RGB: {} \n Depth: {}\n Mask: {}\n ------------- \n'.format(self.rgb_paths[self.index], self.depth_paths[self.index], self.mask_paths[self.index]))

        self.rgb_publisher.publish(self.bridge.cv2_to_imgmsg(rgb))
        self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(depth,  encoding='passthrough'))
        self.mask_publisher.publish(self.bridge.cv2_to_imgmsg(mask))
        self.publish_camera_info()

        self.index += 1


def main():
    rospy.init_node('rgbd_publisher_node', anonymous=True)
    argv = rospy.myargv()
    parser = argparse.ArgumentParser(description='RGBD with mask publisher')
    parser.add_argument('--rgb', help='Path to directory containing RGB data')
    parser.add_argument('--depth', help='Path to directory containing depth data')
    parser.add_argument('--mask', help='Path to directory containing mask data')

    args = parser.parse_args(argv[1:])
    rospy.sleep(5)
    publisher = RgbdPublisher(args.rgb, args.depth, args.mask)

    rospy.spin()


if __name__ == '__main__':
    main()
