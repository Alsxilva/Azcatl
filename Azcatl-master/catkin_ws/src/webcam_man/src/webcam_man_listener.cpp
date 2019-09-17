#include <ros/ros.h>
#include <opencv2/opencv2.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

void callbackImage(const sensor_msgs::CompressedImageConstPtr &msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr=cv_bridge::toCvCopy(msgs,sensor_msgs::image_encoding::BGR8);
	cv::Mat frame = cv->image;
	cv::imshow("Imagen",frame);
	cv::waitKey(27);
}

int main(int argc,char **argv){
	printf("Iniciando webcam_man_listener node ... \n");
	ros::init(argc,argv,"web_cam_listener");
	ros::NodeHandle nh;
	ros::Subscriber subImage = nh.subscribe("/webcam_image/compressed",1,callbackImage)
	ros::Rate rate(30);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}
