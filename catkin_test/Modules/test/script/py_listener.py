#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from test_msgs.msg import Position

def callback(msg):
    rospy.loginfo('Receive: %f, %f', msg.x, msg.y)

def listener():
    rospy.init_node('py_listener', anonymous=True)
    rospy.Subscriber('/test/Position', Position, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInitException:pass