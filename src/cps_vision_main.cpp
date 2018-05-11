#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <cps_vision/cps_vision.h>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>


using namespace cv;
using namespace std;
using namespace cv_projective;

bool freshImage;
bool match;

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

bool findTarget(const cv::Mat &image,cv::Mat &blueImage){
<<<<<<< HEAD
    cv::Mat blueImage1 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage2 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage3 = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat blueImage4 = cv::Mat::zeros(480, 640, CV_8UC1);
//	cv::inRange(image, cv::Scalar(30, 30, 0), cv::Scalar(100,100,0), blueImage1);   // 30, 30, 0   100, 100, 0 object
    cv::inRange(image, cv::Scalar(100, 20, 0), cv::Scalar(200,100,100), blueImage2); //100, 20 ,0  200, 100, 100
    cv::inRange(image, cv::Scalar(200, 100, 0), cv::Scalar(255,200,100), blueImage3);
    cv::inRange(image, cv::Scalar(20, 20, 0), cv::Scalar(100,100,20), blueImage1);

//    threshold(blueImage, blueImage, 200, 255, THRESH_BINARY );
//    blueImage = blueImage1 + blueImage2 + blueImage3 + blueImage4;
    cv::add(blueImage1, blueImage2, blueImage4);
    cv::add(blueImage3, blueImage4, blueImage);
=======
	cv::inRange(image, cv::Scalar(50, 20, 0), cv::Scalar(140,90,20), blueImage);   // 110, 150, 150   255, 0, 0
	ROS_INFO("3");
>>>>>>> c429c7910c27ab9594d6c966c746280b484d1d57
	imshow("Image with only blue pixel", blueImage);
	cv::waitKey();
	ROS_INFO_STREAM("Total "<< cv::countNonZero(blueImage) << "  blue pixels");

	// Need to be determined.
	return cv::countNonZero(blueImage) > 30;
}

cv::Mat matchPattern(char* filenames,const cv::Mat &rawImg ){

    std::vector<Point2f> filtered_pixels;
    cv::Mat position_pixel = cv::Mat::zeros(3, 1, CV_64FC1);

    cv::Mat targetImg = cv::Mat::zeros(480, 640, CV_8UC3);
    targetImg = imread(filenames, IMREAD_GRAYSCALE); //FIXME filename
    Size size(480,640);

    resize(targetImg,targetImg,size);
    if (!targetImg.data || !rawImg.data){
        ROS_INFO_STREAM("Error reading images ");
    }

   // cv::SiftFeatureDetector detector;
    int minHessian = 300; //threshold
    Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    detector->detect(targetImg, keypoints_1);
    detector->detect(rawImg, keypoints_2);

    cv::Mat descriptor_target, descriptor_raw;
    detector->compute(targetImg,keypoints_1, descriptor_target);
    detector->compute(rawImg,keypoints_2, descriptor_raw);

    cv::BFMatcher matcher(NORM_L2);
    std::vector<DMatch> matches;
    std::vector<DMatch> matches_filtered;
    matcher.match(descriptor_target,descriptor_raw,matches,noArray());
    for (int i = 0; i < matches.size(); ++i) {

        if (matches[i].distance<0.1){
            matches_filtered.push_back(matches[i]);
//            ROS_INFO_STREAM("matches: "<<matches[i].distance);
//            ROS_INFO_STREAM("matches: "<<keypoints_2[matches[i].trainIdx].pt);
            filtered_pixels.push_back(keypoints_2[matches[i].trainIdx].pt);
        }
        //ROS_INFO_STREAM("matches: "<<matches_filtered[i].distance);
    }


    for (int j = 0; j < filtered_pixels.size(); ++j) {
        position_pixel.at<double>(0,0) += filtered_pixels[j].x;
        position_pixel.at<double>(1,0) += filtered_pixels[j].y;

    }

    if(filtered_pixels.size() > 0){
        position_pixel.at<double>(0,0) /= filtered_pixels.size();
        position_pixel.at<double>(1,0) /= filtered_pixels.size();
        position_pixel.at<double>(2,0) = 1;
        match = true;
    }else{match = false;}

    cv::Mat match_mat;
    cv::drawMatches(targetImg, keypoints_1,rawImg,keypoints_2,matches_filtered,match_mat);
    imshow("matches image", match_mat);
    cv::waitKey();

    return position_pixel;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;

    ros::Duration(2).sleep();
    CPSVision CPSVision(&nh);
    freshImage = false;
    match = false;

    //get image size from camera model, or initialize segmented images,
    cv::Mat raw_image = cv::Mat::zeros(480, 640, CV_8UC3);//this is 3 channel image

//    raw_image = imread("/home/ranhao/ros_ws/src/cps_vision/new2.jpg",IMREAD_COLOR);
//    Size size(480,640);
//    resize(raw_image,raw_image,size);
//    cv::imshow("raw image ", raw_image);
//    cv::waitKey(10);
////    for(int row = 0; row < raw_image.rows; row++){
////        for(int col = 0; col < raw_image.cols; col++){
////            Vec3b pixel = raw_image.at<Vec3b>(row,col);
////            ROS_INFO_STREAM("RGB c1: "<< (int)pixel.val[0]);
////            ROS_INFO_STREAM("RGB c2: "<< (int)pixel.val[1]);
////            ROS_INFO_STREAM("RGB c3: "<< (int)pixel.val[2]);
////        }
////    }
//
//    cv::Mat blueImage = cv::Mat::zeros(480, 640, CV_8UC1);
//    findTarget(raw_image, blueImage);
//    matchPattern("/home/ranhao/ros_ws/src/cps_vision/object.jpg",blueImage);
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

<<<<<<< HEAD
=======
			// seg_image = segmentation(raw_image); //segmentation here
ROS_INFO("HERE");
>>>>>>> c429c7910c27ab9594d6c966c746280b484d1d57
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
                CPSVision.P1_mat = matchPattern("/home/ranhao/ros_ws/src/cps_vision/object.jpg",blueImage);
            	CPSVision.getG1();
                ros::Duration(1).sleep();

				// take the second picture.
                ros::spinOnce();
				cv::cvtColor(raw_image, raw_image, CV_BGR2RGB);

                if(findTarget(raw_image, blueImage)) {
					ROS_INFO("target found 2");
                    CPSVision.P2_mat = matchPattern("/home/ranhao/ros_ws/src/cps_vision/object.jpg",blueImage);
                    CPSVision.getG2();
                    if(match){ //have keypoints matched
                        cv::Mat W_pose = CPSVision.computePose();
                    }
					// publish.....
                }else{
					ROS_INFO("cannot find target in the 2nd image stream");
				}
			}

			freshImage = false;
		}

	}

	return 0;
}
