#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pub_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image_topic", 10);
    Mat image = imread("/home/kavin/Pictures/ubuntu.png", IMREAD_COLOR);
    if (image.empty())
    {
        cout << "can't find picture" << endl;
        return -1;
    }
    for (int i = 5; i < 50; i++) {          // 行
        for (int j = 30; j < 80; j++) {     // 列
            image.at<Vec3b>(i, j)[0] = 255;
            image.at<Vec3b>(i, j)[1] = 200;
            image.at<Vec3b>(i, j)[2] = 0;
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate loop_rate(1);
    while(nh.ok())
    {
        image_pub.publish(msg);
        ROS_WARN("publish an image");
        loop_rate.sleep();
    }
    return 0;
}