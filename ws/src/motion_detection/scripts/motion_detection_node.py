#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class MotionDetectionNode:
    def __init__(self):
        self.sub = rospy.Subscriber("image", Image, self.image_callback)
        self.cv_bridge = CvBridge()

    def image_callback(self, image_msg):
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('motion_detection_node', anonymous=True)
    motion_detection_node = MotionDetectionNode()
    rospy.spin()