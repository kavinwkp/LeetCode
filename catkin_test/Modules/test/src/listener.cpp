#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include "test/test_class.h"

// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
// #include <cv_bridge/cv_bridge.h>

ros::Subscriber data1_sub;
ros::Subscriber data2_sub;
ros::Publisher ret_pub;

double res1;
double res2;
double sum;
float rate_hz = 5;

void data1_cb(const std_msgs::Float64::ConstPtr &data1);
void data2_cb(const std_msgs::Float64::ConstPtr &data2);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    // nh.param<float>("rate_hz", rate_hz, 5);
    ros::Rate loop_rate(rate_hz);

    ret_pub = nh.advertise<std_msgs::Float64>("/result", 10);
    std_msgs::Float64 result;

    data1_sub = nh.subscribe("/test/data1", 10, data1_cb);
    data2_sub = nh.subscribe("/test/data2", 10, data2_cb);

    test_class _test_class;

    while (ros::ok()) {
        ros::spinOnce();
        sum = res1 * res1 + res2 * res2;
        ROS_WARN("Sum: %f", sum);
        // _test_class.info();
        result.data = sum;
        ret_pub.publish(result);
        loop_rate.sleep();
    }
}

void data1_cb(const std_msgs::Float64::ConstPtr &data1)
{
    ROS_INFO("Receive data1: %f", data1->data);
    res1 = data1->data;
}

void data2_cb(const std_msgs::Float64::ConstPtr &data2)
{
    ROS_INFO("Receive data2: %f", data2->data);
    res2 = data2->data;
}