#include <cps_vision/cps_vision.h>  //everything inside here
using namespace std;

CPSVision::CPSVision(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle){

	// ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	// ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);

	projectionMat_subscriber = node_handle.subscribe("/davinci_endo/unsynched/right/camera_info", 1, &CPSVision::projectionMatCB, this);

	raw_image = cv::Mat::zeros(480, 640, CV_8UC3);
	projection_mat = cv::Mat::zeros(3,4,CV_64FC1);

	freshCameraInfo = false;

};

CPSVision::~CPSVision() {
};

void CPSVision::projectionMatCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight){

}

