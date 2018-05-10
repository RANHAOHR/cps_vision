#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cps_vision/cps_vision.h>

using namespace cv;
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

bool findTarget(const cv::Mat &image,cv::Mat &blueImage){
	cv::inRange(image, cv::Scalar(30, 20, 0), cv::Scalar(150,100,20), blueImage);   // 110, 150, 150   255, 0, 0
	ROS_INFO("3");
	imshow("Image with only blue pixel", blueImage);
	cv::waitKey();
	ROS_INFO_STREAM("Total "<< cv::countNonZero(blueImage) << "  blue pixels");

	// Need to be determined.
	return cv::countNonZero(blueImage) > 10;
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	//Initialize particles
	ros::Duration(2).sleep();

	CPSVision CPSVision(&nh);

    freshImage = false;

    cv::Mat seg_image  = cv::Mat::zeros(480, 640, CV_8UC1); //this is 1 channel image 

    //get image size from camera model, or initialize segmented images,
    cv::Mat raw_image = cv::Mat::zeros(480, 640, CV_8UC3);//this is 3 channel image

    // get the image that contains only blue pixels.
    cv::Mat blueImage = cv::Mat::zeros(480, 640, CV_8UC1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe(
            "/camera/rgb/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &raw_image)));

    ROS_INFO("---- done subscribe -----");
    ros::Duration(1).sleep();

	while (nh.ok()) {
		ros::spinOnce();
		// if camera is ready, track segmented image
		if (freshImage) {
			ros::spinOnce();

			// seg_image = segmentation(raw_image); //segmentation here
			cv::cvtColor(raw_image, raw_image, CV_BGR2RGB);
			//show what you see......
			cv::imshow("raw image ", raw_image);
			// cv::imshow("seg image ", seg_image);
            cv::waitKey(10);

		//	for(int row = 0; row < raw_image.rows; row++){
		//		for(int col = 0; col < raw_image.cols; col++){
		//			Vec3b pixel = raw_image.at<Vec3b>(row,col);
		//			ROS_INFO_STREAM("RGB c1: "<< (int)pixel(1));
		//			ROS_INFO_STREAM("RGB c2: "<< (int)pixel(2));
		//			ROS_INFO_STREAM("RGB c3: "<< (int)pixel(3));
		//		}
		//	}
		//	/*when getting new image, do somthing*/
			if(findTarget(raw_image, blueImage)){
				ROS_INFO("target found 1");
            	CPSVision.getG1();
				CPSVision.getLocation1(blueImage);
                ros::Duration(1).sleep();

				// take the second picture.
                ros::spinOnce();
				cv::cvtColor(raw_image, raw_image, CV_BGR2RGB);

                if(findTarget(raw_image, blueImage)) {
					ROS_INFO("target found 2");
                    CPSVision.getG2();
                    CPSVision.getLocation2(blueImage);

					cv::Mat W_pose = CPSVision.computePose();
					// publish.....
                }else{
					ROS_INFO("cannot find target in the 2nd image stream");
				}
			}

			freshImage = false;
		}
			// ROS_INFO_STREAM("raw_image"<<raw_image);

	}
	return 0;
}
