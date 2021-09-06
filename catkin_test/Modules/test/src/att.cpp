#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

ros::Publisher N_pose_pub;
ros::Publisher B_pose_pub;
ros::Publisher T_pose_pub;
ros::Publisher M_pose_pub;
ros::Publisher BT_pose_pub;

Quaterniond get_err(const Vector3d& src, const Vector3d& dst);

int main(int argc, char **argv) {
	ros::init(argc, argv, "att");
	ros::NodeHandle nh;
	N_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("Earth_pose", 10);
	B_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("Body_pose", 10);
	T_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("Target_pose", 10);
	M_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("Middle_pose", 10);
	BT_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("BodyToTarget_pose", 10);

	geometry_msgs::PoseStamped N_pose;
	N_pose.header.stamp = ros::Time::now();
	N_pose.header.frame_id = "world";
	N_pose.pose.position.x = 0;
	N_pose.pose.position.y = 0;
	N_pose.pose.position.z = 0;
	N_pose.pose.orientation.x = 0;
	N_pose.pose.orientation.y = 0;
	N_pose.pose.orientation.z = 0;
	N_pose.pose.orientation.w = 1;

	geometry_msgs::PoseStamped B_pose;
	B_pose.header.stamp = ros::Time::now();
	B_pose.header.frame_id = "world";
	B_pose.pose.position.x = 0;
	B_pose.pose.position.y = 0;
	B_pose.pose.position.z = 0;
	B_pose.pose.orientation.x = 0;
	B_pose.pose.orientation.y = 0;
	B_pose.pose.orientation.z = 0.5;
	B_pose.pose.orientation.w = 0.866;

	geometry_msgs::PoseStamped T_pose;
	T_pose.header.stamp = ros::Time::now();
	T_pose.header.frame_id = "world";
	T_pose.pose.position.x = 0;
	T_pose.pose.position.y = 0;
	T_pose.pose.position.z = 0;
	T_pose.pose.orientation.x = 0.707;
	T_pose.pose.orientation.y = 0;
	T_pose.pose.orientation.z = 0;
	T_pose.pose.orientation.w = 0.707;

	Quaterniond qB;
	qB.x() = B_pose.pose.orientation.x;
	qB.y() = B_pose.pose.orientation.y;
	qB.z() = B_pose.pose.orientation.z;
	qB.w() = B_pose.pose.orientation.w;

	MatrixXd mat_B = qB.toRotationMatrix();
    Vector3d e_z_B = mat_B * Vector3d(0, 0, 1);
    cout << e_z_B << endl;

	Quaterniond qT;
	qT.x() = T_pose.pose.orientation.x;
	qT.y() = T_pose.pose.orientation.y;
	qT.z() = T_pose.pose.orientation.z;
	qT.w() = T_pose.pose.orientation.w;
	MatrixXd mat_T = qT.toRotationMatrix();
    Vector3d e_z_T = mat_T * Vector3d(0, 0, 1);
    cout << e_z_T << endl;

	Quaterniond qBM_N = get_err(e_z_B, e_z_T);     // B -> M (N)
	cout << qBM_N.coeffs() << endl;

	Quaterniond qNM_N = qBM_N * qB;		// N -> M (N)

	geometry_msgs::PoseStamped M_pose;
	M_pose.header.stamp = ros::Time::now();
	M_pose.header.frame_id = "world";
	M_pose.pose.position.x = 0;
	M_pose.pose.position.y = 0;
	M_pose.pose.position.z = 0;
	M_pose.pose.orientation.x = qNM_N.x();
	M_pose.pose.orientation.y = qNM_N.y();
	M_pose.pose.orientation.z = qNM_N.z();
	M_pose.pose.orientation.w = qNM_N.w();

	Quaterniond qMT_M = qNM_N.inverse() * qT;    // M -> B (M)
	double _yaw_w = 0.5;
    Quaterniond qTT = qNM_N * Quaterniond(cosf(_yaw_w * acosf(qMT_M.w())), 0, 0, sinf(_yaw_w * asinf(qMT_M.z())));
    cout << qTT.coeffs() << endl;	// equal to qT in theory

	geometry_msgs::PoseStamped BT_pose;
	BT_pose.header.stamp = ros::Time::now();
	BT_pose.header.frame_id = "world";
	BT_pose.pose.position.x = 0;
	BT_pose.pose.position.y = 0;
	BT_pose.pose.position.z = 0;
	BT_pose.pose.orientation.x = qTT.x();
	BT_pose.pose.orientation.y = qTT.y();
	BT_pose.pose.orientation.z = qTT.z();
	BT_pose.pose.orientation.w = qTT.w();

	ros::Rate rate(5);
	while (ros::ok()) {
		ROS_WARN("Pose Publishing");
		N_pose_pub.publish(N_pose);
		B_pose_pub.publish(B_pose);
		T_pose_pub.publish(T_pose);
		M_pose_pub.publish(M_pose);
		BT_pose_pub.publish(BT_pose);
		rate.sleep();
	}
}

Quaterniond get_err(const Vector3d& src, const Vector3d& dst) {
    Quaterniond q;
    Vector3d cr = src.cross(dst);
    const float dt = src.dot(dst);
    q.w() = dt + sqrt(src.squaredNorm() * dst.squaredNorm());
    q.x() = cr(0);
    q.y() = cr(1);
    q.z() = cr(2);
    q.normalize();
    return q;
}