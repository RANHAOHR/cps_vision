#include <cps_vision/cps_vision.h>  //everything inside here
#include <opencv/cv.hpp>

using namespace std;

CPSVision::CPSVision(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle){
    projectionMat_subscriber = node_handle.subscribe("/camera/rgb/camera_info", 1, &CPSVision::projectionMatCB, this);
      pose_subscriber = node_handle.subscribe("/mavros/local_position/odom", 1, &CPSVision::getPose, this);

    raw_image = cv::Mat::zeros(480, 640, CV_8UC3);

    C_mat = cv::Mat::zeros(3,4, CV_64FC1);

    P1_mat = cv::Mat::zeros(3, 1, CV_32FC1);
    P2_mat = cv::Mat::zeros(3, 1, CV_32FC1);

    R_mat = cv::Mat::zeros(3, 3, CV_64FC1);
    T_mat = cv::Mat::zeros(3, 1, CV_64FC1);
    G1_mat = cv::Mat::eye(4,4, CV_64FC1);
    G2_mat = cv::Mat::eye(4,4, CV_64FC1);

    Gc_mat = (cv::Mat_<double>(4,4) << 0,   1,   0,   0,
	    					  1,    0,   0,    0.000,
	    					  0,   0,   -1,    -0.04,
	    						0, 	   0,  0, 	1);

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
    //ROS_INFO_STREAM("C MATRIX"<<C_mat);

}

void CPSVision::getPose(const nav_msgs::Odometry::ConstPtr &pose) {
    cv::Mat vect3 = cv::Mat::zeros(3,1,CV_64FC1);
    double angle = 2*acos(pose->pose.pose.orientation.w);
    vect3.at<double>(0,0) = angle * pose->pose.pose.orientation.x / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    vect3.at<double>(1,0) = angle * pose->pose.pose.orientation.y / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    vect3.at<double>(2,0) = angle * pose->pose.pose.orientation.z / sqrt(1- pose->pose.pose.orientation.w * pose->pose.pose.orientation.w);
    cv::Rodrigues(vect3, R_mat);

    T_mat.at<double>(0,0) = pose->pose.pose.position.x;
    T_mat.at<double>(1,0) = pose->pose.pose.position.y;
    T_mat.at<double>(2,0) = pose->pose.pose.position.z;
}


void CPSVision::getG1() {
    R_mat.copyTo(G1_mat.colRange(0, 3).rowRange(0, 3));
    T_mat.copyTo(G1_mat.colRange(3, 4).rowRange(0, 3));

}

void CPSVision::getG2() {

  R_mat.copyTo(G2_mat.colRange(0, 3).rowRange(0, 3));
  T_mat.copyTo(G2_mat.colRange(3, 4).rowRange(0, 3));
}

cv::Mat CPSVision::computePose() {
    cv::Mat P_mat = C_mat * Gc_mat * G1_mat;
    cv::Mat Q_mat = C_mat * Gc_mat * G2_mat;
    cv::Mat A_mat = cv::Mat::zeros(4, 4, CV_64FC1);

    float u1 = P1_mat.at<float>(0);
    float v1 = P1_mat.at<float>(1);
    cv::Mat temp_mat = u1 * P_mat.colRange(0, 4).rowRange(2, 3) - P_mat.colRange(0, 4).rowRange(0, 1);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(0, 1));

    temp_mat = v1 * P_mat.colRange(0, 4).rowRange(2, 3) - P_mat.colRange(0, 4).rowRange(1, 2);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(1, 2));

    float u2 = P2_mat.at<float>(0);
    float v2 = P2_mat.at<float>(1);
    temp_mat = u2 * Q_mat.colRange(0, 4).rowRange(2, 3) - Q_mat.colRange(0, 4).rowRange(0, 1);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(2, 3));
    temp_mat = v2 * Q_mat.colRange(0, 4).rowRange(2, 3) - Q_mat.colRange(0, 4).rowRange(1, 2);
    temp_mat.copyTo(A_mat.colRange(0, 4).rowRange(3, 4));

    cv::Mat target_position = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat resulting_position = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::SVD svd = cv::SVD(A_mat);
    resulting_position = svd.vt.colRange(3, 4).rowRange(0, 4);
    double scale_w = resulting_position.at<double>(3, 0);
    target_position = resulting_position / scale_w;
    ROS_INFO_STREAM("Reported posistion is: " << target_position);

    return target_position;
}
