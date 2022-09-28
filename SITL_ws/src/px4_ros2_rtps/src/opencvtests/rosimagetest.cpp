#/*
File: rosimagetest.cpp
*/

#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <sensor_msgs/image_encodings.hpp>
#include "image_transport/image_transport.hpp"

using namespace std;
using namespace cv;
using std::placeholders::_1;

/**************** OpenCV global variables *****************/

cv::VideoCapture in_video;
cv::Mat image;

/************ ROS2 Node ************/

class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("ros_camera_receiver") {
        image_sub = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image/compressed", 10, std::bind(&DebugVectAdvertiser::imageCallback, this, _1));
	}
private:
	void imageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) const
		{
		try {
			cv::imshow("view", cv_bridge::toCvCopy(msg, "bgr8")->image);
			cv::waitKey(3);
		} catch (cv_bridge::Exception & e) {
			auto logger = rclcpp::get_logger("my_subscriber");
			/*RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());*/
		}
		};
	rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub;   
};


int main(int argc, char* argv[])
{
	std::cout << "Video input received!" << std::endl;
	std::cout << "Starting ArUco autoland control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    rclcpp::shutdown();
    return 0;
}