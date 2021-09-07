#! /usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


rospy.init_node('py_image_sub', anonymous=True)

def colorCallBack(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("color", img)
    cv2.waitKey(10)

def listener():
    rospy.Subscriber("image_topic", Image, colorCallBack)
    rospy.spin()


if __name__ == '__main__':
    print(cv2.__version__)
    try:
        listener()
    except rospy.ROSInterruptException:pass
    print('done')