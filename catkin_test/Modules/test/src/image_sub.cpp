#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;

std_msgs::Header imageHeader_;
image_transport::Subscriber imageSubscriber;
void cameraCallBack(const sensor_msgs::ImageConstPtr& msg) {
    ROS_WARN("receive image");
    cv_bridge::CvImagePtr cam_image;
    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge except: %s", e.what());
        return;
    }
    if (cam_image) {
        cv::Mat cv_image = cam_image->image.clone();
        ROS_WARN("resolution: %d x %d", cv_image.cols, cv_image.rows);
    }
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_sub_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(10);
    imageSubscriber = it.subscribe("image_topic", 1, cameraCallBack);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
