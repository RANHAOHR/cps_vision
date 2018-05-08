#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cps_vision/cps_vision.h>

using namespace std;
using namespace cv_projective;

bool freshImage;

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		//cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
		//outputImage[0] = src.clone();
		cv_ptr = cv_bridge::toCvCopy(msg);
		outputImage[0] = cv_ptr->image;
		freshImage = true;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

}

cv::Mat segmentation(cv::Mat &InputImg) {

	cv::Mat src, src_gray;
	cv::Mat grad;

	cv::Mat res;
	src = InputImg;

	resize(src, src, cv::Size(), 1, 1);

	double lowThresh = 20;

	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	blur(src_gray, src_gray, cv::Size(3, 3));

	Canny(src_gray, grad, lowThresh, 4 * lowThresh, 3); //use Canny segmentation

	grad.convertTo(res, CV_32FC1);

	return res;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	//Initialize particles
	ros::Duration(10).sleep();

	CPSVision CPSVision(&nh);

    freshImage = false;

    cv::Mat seg_image  = cv::Mat::zeros(480, 640, CV_8UC1);

    //get image size from camera model, or initialize segmented images,
    cv::Mat raw_image = cv::Mat::zeros(480, 640, CV_8UC3);//CV_32FC1

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe(
            "/davinci_endo/left/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &raw_image)));

    ROS_INFO("---- done subscribe -----");
    ros::Duration(2).sleep();

	while (nh.ok()) {
		ros::spinOnce();

		// if camera is ready, track segmented image
		if (freshImage) {

			/*when getting new iamge, do somthing*/

			freshImage = false;
			}

	}

	return 0;
}
