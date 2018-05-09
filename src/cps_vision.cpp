#include <cps_vision/cps_vision.h>  //everything inside here
#include <opencv-3.3.1/opencv/cv.hpp>

using namespace std;

CPSVision::CPSVision(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle){

	// ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	// ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);

	projectionMat_subscriber = node_handle.subscribe("/davinci_endo/unsynched/right/camera_info", 1, &CPSVision::projectionMatCB, this);

    //gMat_subscriber = node_handle.subscribe("/davinci_endo/unsynched/right/camera_info", 1, &CPSVision::getPosition, this);

	raw_image = cv::Mat::zeros(480, 640, CV_8UC3);

	C_mat = cv::Mat::zeros(3,4, CV_64FC1);

	P1_mat = cv::Mat::zeros(3, 1, CV_64FC1);
    P2_mat = cv::Mat::zeros(3, 1, CV_64FC1);

    R_mat = cv::Mat::zeros(3, 3, CV_64FC1);
    T_mat = cv::Mat::zeros(3, 1, CV_64FC1);
    G1_mat = cv::Mat::zeros(4,4, CV_64FC1);
    G2_mat = cv::Mat::zeros(4,4, CV_64FC1);

	freshCameraInfo = false;

};

CPSVision::~CPSVision() {
};

void CPSVision::projectionMatCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight){

	C_mat.at<double>(0,0) = projectionRight->P[0];
	C_mat.at<double>(0,1) = projectionRight->P[1];
	C_mat.at<double>(0,2) = projectionRight->P[2];
	C_mat.at<double>(0,3) = projectionRight->P[3];

	C_mat.at<double>(1,0) = projectionRight->P[4];
	C_mat.at<double>(1,1) = projectionRight->P[5];
	C_mat.at<double>(1,2) = projectionRight->P[6];
	C_mat.at<double>(1,3) = projectionRight->P[7];

	C_mat.at<double>(2,0) = projectionRight->P[8];
	C_mat.at<double>(2,1) = projectionRight->P[9];
	C_mat.at<double>(2,2) = projectionRight->P[10];
	C_mat.at<double>(2,3) = projectionRight->P[11];
	ROS_INFO_STREAM("C MATRIX"<<C_mat);

}


void CPSVision::getLocation1(const cv::Mat &image){
	cv::Mat sum_mat = cv::Mat::zeros(1, 3, CV_64FC1);
	int count = 0;
	for(int row = 0; row < image.rows; row++){
		for(int col = 0; col < image.cols; col++){
			if(image.at<double>(row, col) != 0){
				sum_mat.at<double>(0) += row;
				sum_mat.at<double>(1) += col;
				count++;
			}
		}
	}
	P1_mat.at<double>(0) = sum_mat.at<double>(0) / count;
	P1_mat.at<double>(1) = sum_mat.at<double>(1) / count;
	P1_mat.at<double>(2) = 1;
    ROS_INFO_STREAM("P1 MATRIX"<<P1_mat);
}
void CPSVision::getLocation2(const cv::Mat &image){
    cv::Mat sum_mat = cv::Mat::zeros(1, 3, CV_64FC1);
    int count = 0;
    for(int row = 0; row < image.rows; row++){
        for(int col = 0; col < image.cols; col++){
            if(image.at<double>(row, col) != 0){
                sum_mat.at<double>(0) += row;
                sum_mat.at<double>(1) += col;
                count++;
            }
        }
    }
    P2_mat.at<double>(0) = sum_mat.at<double>(0) / count;
    P2_mat.at<double>(1) = sum_mat.at<double>(1) / count;
    P2_mat.at<double>(2) = 1;
    ROS_INFO_STREAM("P2 MATRIX"<<P2_mat);
}

void CPSVision::getPosition(const nav_msgs::Odometry::ConstPtr &positon) {
    cv::Mat vect3 = cv::Mat::zeros(1,3,CV_64FC1);
    double angle = 2*acos(positon->pose.pose.orientation.w);
    vect3.at<double>(0,0) = angle * positon->pose.pose.orientation.x / sqrt(1- sqrt(positon->pose.pose.orientation.w));
    vect3.at<double>(1,0) = angle * positon->pose.pose.orientation.y / sqrt(1- sqrt(positon->pose.pose.orientation.w));
    vect3.at<double>(2,0) = angle * positon->pose.pose.orientation.z / sqrt(1- sqrt(positon->pose.pose.orientation.w));
    cv::Rodrigues(vect3, R_mat);

    T_mat.at<double>(0,0) = positon->pose.pose.position.x;
    T_mat.at<double>(1,0) = positon->pose.pose.position.y;
    T_mat.at<double>(2,0) = positon->pose.pose.position.z;
}


void CPSVision::getG1() {
    G1_mat.at<double>(0,0) = R_mat.at<double>(0,0);
    G1_mat.at<double>(0,1) = R_mat.at<double>(0,1);
    G1_mat.at<double>(0,2) = R_mat.at<double>(0,2);

    G1_mat.at<double>(1,0) = R_mat.at<double>(1,0);
    G1_mat.at<double>(1,1) = R_mat.at<double>(1,1);
    G1_mat.at<double>(1,2) = R_mat.at<double>(1,2);

    G1_mat.at<double>(2,0) = R_mat.at<double>(2,0);
    G1_mat.at<double>(2,1) = R_mat.at<double>(2,1);
    G1_mat.at<double>(2,2) = R_mat.at<double>(2,2);

    G1_mat.at<double>(0,3) = T_mat.at<double>(0,0);
    G1_mat.at<double>(1,3) = T_mat.at<double>(1,0);
    G1_mat.at<double>(2,3) = T_mat.at<double>(2,0);

    G1_mat.at<double>(3,3) = 1;
}

void CPSVision::getG2() {
    G2_mat.at<double>(0,0) = R_mat.at<double>(0,0);
    G2_mat.at<double>(0,1) = R_mat.at<double>(0,1);
    G2_mat.at<double>(0,2) = R_mat.at<double>(0,2);

    G2_mat.at<double>(1,0) = R_mat.at<double>(1,0);
    G2_mat.at<double>(1,1) = R_mat.at<double>(1,1);
    G2_mat.at<double>(1,2) = R_mat.at<double>(1,2);

    G2_mat.at<double>(2,0) = R_mat.at<double>(2,0);
    G2_mat.at<double>(2,1) = R_mat.at<double>(2,1);
    G2_mat.at<double>(2,2) = R_mat.at<double>(2,2);

    G2_mat.at<double>(0,3) = T_mat.at<double>(0,0);
    G2_mat.at<double>(1,3) = T_mat.at<double>(1,0);
    G2_mat.at<double>(2,3) = T_mat.at<double>(2,0);

    G2_mat.at<double>(3,3) = 1;
}

cv::Mat CPSVision::computePose() {
    cv::Mat P_mat = C_mat * G1_mat;
    cv::Mat Q_mat = C_mat * G2_mat;
    cv::Mat A_mat = cv::Mat::zeros(4, 4, CV_64FC1);
    A_mat.at<double>(0,0) = P1_mat.at<double>(0) * P_mat.at<double>(2,0) - P_mat.at<double>(0,0);
    A_mat.at<double>(0,1) = P1_mat.at<double>(0) * P_mat.at<double>(2,1) - P_mat.at<double>(0,1);
    A_mat.at<double>(0,2) = P1_mat.at<double>(0) * P_mat.at<double>(2,2) - P_mat.at<double>(0,2);
    A_mat.at<double>(0,3) = P1_mat.at<double>(0) * P_mat.at<double>(2,3) - P_mat.at<double>(0,3);
    A_mat.at<double>(1,0) = P1_mat.at<double>(1) * P_mat.at<double>(2,0) - P_mat.at<double>(1,0);
    A_mat.at<double>(1,1) = P1_mat.at<double>(1) * P_mat.at<double>(2,1) - P_mat.at<double>(1,1);
    A_mat.at<double>(1,2) = P1_mat.at<double>(1) * P_mat.at<double>(2,2) - P_mat.at<double>(1,2);
    A_mat.at<double>(1,3) = P1_mat.at<double>(1) * P_mat.at<double>(2,3) - P_mat.at<double>(1,3);
    A_mat.at<double>(2,0) = P2_mat.at<double>(0) * P_mat.at<double>(2,0) - P_mat.at<double>(0,0);
    A_mat.at<double>(2,1) = P2_mat.at<double>(0) * P_mat.at<double>(2,1) - P_mat.at<double>(0,1);
    A_mat.at<double>(2,2) = P2_mat.at<double>(0) * P_mat.at<double>(2,2) - P_mat.at<double>(0,2);
    A_mat.at<double>(2,3) = P2_mat.at<double>(0) * P_mat.at<double>(2,3) - P_mat.at<double>(0,3);
    A_mat.at<double>(3,0) = P2_mat.at<double>(1) * P_mat.at<double>(2,0) - P_mat.at<double>(1,0);
    A_mat.at<double>(3,1) = P2_mat.at<double>(1) * P_mat.at<double>(2,1) - P_mat.at<double>(1,1);
    A_mat.at<double>(3,2) = P2_mat.at<double>(1) * P_mat.at<double>(2,2) - P_mat.at<double>(1,2);
    A_mat.at<double>(3,3) = P2_mat.at<double>(1) * P_mat.at<double>(2,3) - P_mat.at<double>(1,3);

    cv::Mat X_mat = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::SVD svd = cv::SVD(A_mat);
    X_mat.at<double>(0,0) = svd.vt.at<double>(0,3);
    X_mat.at<double>(1,0) = svd.vt.at<double>(1,3);
    X_mat.at<double>(2,0) = svd.vt.at<double>(2,3);
    X_mat.at<double>(3,0) = svd.vt.at<double>(3,3);
    return X_mat;
}