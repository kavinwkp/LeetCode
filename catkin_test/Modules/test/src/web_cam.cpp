#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;

image_transport::Publisher image_pub;

int main(int argc, char **argv) {
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(30);
    image_pub = it.advertise("/camera/image_raw", 1);
    cv::VideoCapture cap(1);
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    bool first_in = true;
    while (ros::ok()) {
        cap >> frame;
        if (!frame.empty()) {
            if (first_in) {
                cout << "resolution: " << frame.cols << "x" << frame.rows << endl;
                first_in = false;
            }
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    cap.release();
}
