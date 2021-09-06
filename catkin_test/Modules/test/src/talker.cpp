#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
// #include <geometry_msgs/PoseStamped.h>
#include "test_msgs/Position.h"
#include "test/simple.h"

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
// #include <cv_bridge/cv_bridge.h>

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pos_pub;

void pub_data(std_msgs::Float64& data1, std_msgs::Float64& data2);

int main(int argc, char** argv) {
    ros::init(argc, argv, "talker");    // 节点名
    ros::NodeHandle nh;                  // 句柄
    pub1 = nh.advertise<std_msgs::Float64>("/test/data1", 10);
    pub2 = nh.advertise<std_msgs::Float64>("/test/data2", 10);
    pos_pub = nh.advertise<test_msgs::Position>("/test/Position", 10);


    std_msgs::Float64 data1;
    data1.data = 0.0;
    std_msgs::Float64 data2;
    data2.data = 0.0;

    test_msgs::Position pos;
    pos.x = 10;
    pos.y = 20;

    simple _simple(100, 200);

    ros::Rate loop_rate(2);
    while (ros::ok()) {
        std::cout << ">>>>>>>>>>>>>>>>>> Running <<<<<<<<<<<<<<<<<" << std::endl;
        _simple.add();
        _simple.disp();
        data1.data += 0.1;
        std::cout << data1.data << std::endl;
        data2.data += 1.0;
        std::cout << data2.data << std::endl;
        pub_data(data1, data2);
        pos.x += 1;
        pos.y += 1;
        ROS_WARN("Position: (%f, %f)", pos.x, pos.y);
        pos_pub.publish(pos);
        loop_rate.sleep();
    }
}

void pub_data(std_msgs::Float64& data1, std_msgs::Float64& data2) {
    pub1.publish(data1);
    pub2.publish(data2);
}