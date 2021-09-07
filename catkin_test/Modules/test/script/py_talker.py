#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from test_msgs.msg import Position


def talker():
    rospy.init_node('py_talker', anonymous=True)
    Pos_pub = rospy.Publisher('/test/Position', Position, queue_size=10)
    rate = rospy.Rate(10)
    pos = Position()
    pos.x = 10
    pos.y = 20
    while not rospy.is_shutdown():
        pos.x += 1
        pos.y += 2
        rospy.loginfo(pos)
        Pos_pub.publish(pos)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:pass