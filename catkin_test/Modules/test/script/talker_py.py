#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from test_msgs.msg import Position

def talker():
    rospy.init_node('talker_node_py', anonymous=True)
    pub = rospy.Publisher('hello', String, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()   
        rospy.loginfo(str)
        pub.publish(str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:pass