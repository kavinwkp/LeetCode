#ifndef TEST_CLASS_H
#define TEST_CLASS_H

#include "test_msgs/Position.h"
#include <iostream>

using namespace std;

class test_class {
public:
    test_class(void) : test_nh("~") {
        pos_sub = test_nh.subscribe<test_msgs::Position>("/test/Position", 10, &test_class::pos_cb, this);
    }
    void info();

private:
    ros::NodeHandle test_nh;
    ros::Subscriber pos_sub;
    test_msgs::Position pos;

    void pos_cb(const test_msgs::Position::ConstPtr& msg);
};

void test_class::pos_cb(const test_msgs::Position::ConstPtr& msg) {
    pos = *msg;
}

void test_class::info() {
    ROS_INFO("Receive Position: (%f, %f)", pos.x, pos.y);
}

#endif