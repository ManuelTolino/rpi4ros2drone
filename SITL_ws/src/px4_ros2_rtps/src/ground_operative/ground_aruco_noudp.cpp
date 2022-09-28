#/*
File: ground_aruco_noudp.cpp
Author: Manuel Tolino Contreras
*/

#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <more_interfaces/msg/devfeedback.hpp>

#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/****************** GSTREAMER **************/
//pipeline parameters
int device = 0;                 //0=raspicam 1=usb webcam (when both are connected)
int capture_width = 640 ;
int capture_height = 480 ;
int framerate = 30 ;
int display_width = 640 ;
int display_height = 480 ;

// For writer if required
//	int bitrate = 1024;

std::string gstreamer_pipeline(int device, int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " v4l2src device=/dev/video"+ std::to_string(device) + " !"
            " video/x-raw,"
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " queue! videorate! videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! queue ! appsink drop=1";
}

std::string rxwebcam = gstreamer_pipeline(device,
                                          capture_width, capture_height, framerate,
                                          display_width, display_height);
//string rxudp = "udpsrc port=5600 timeout=500 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1";
string rxudp = "udpsrc port=5600 ! tsparse ! tsdemux ! h264parse ! queue ! avdec_h264 ! queue ! videoconvert ! queue ! appsink drop=1 sync=false";
//gst-launch-1.0 udpsrc port=5600 ! tsparse ! tsdemux ! h264parse ! avdec_h264 ! videoconvert ! ximagesink sync=false
string rxdummy = "videotestsrc ! video/x-raw, framerate=20/1 ! videoscale ! videoconvert ! appsink";
string pipeline;
/**************** OpenCV global variables *****************/
cv::Mat image_copy2 ;
cv::VideoCapture cap;
cv::Mat image_black(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

/**************** OpenCV parameters *****************/

/************  ************/

/************ ROS2 Node ************/

auto irlock_msg = px4_msgs::msg::IrlockReport();
auto feedback_data = more_interfaces::msg::Devfeedback();
float rx_devx = 0;
float rx_devy = 0;
float altitude_agl = -1.0;
float distance_quality = -1.0;
int screen_height = 480 ;
int screen_width = 640 ;
int app_heart_beat = 0;
int rx_heartbeat = 0;
int navstate = -1;
//int noUDPflag = 1;

bool big_marker_rx = false ;
bool small_marker_rx = false ;
class GroundAruco : public rclcpp::Node
{
public:
	GroundAruco()
	: Node("ground_ros2_station")
	{
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		irlock_sub_ =
			this->create_subscription<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10,
				[this](const px4_msgs::msg::IrlockReport::UniquePtr irlockreport_msg) {
					rx_devx = irlockreport_msg->pos_x;
					rx_devy = irlockreport_msg->pos_x;
				});
		
		feedback_sub_ =
			this->create_subscription<more_interfaces::msg::Devfeedback>("aruco_feedback", 10,
				[this](const more_interfaces::msg::Devfeedback::UniquePtr feedback_data) {
					for (int i=0; i<=7 ;i++) {
						big_marker_array[i] = feedback_data->big_marker_corners[i];
					};
					for (int i=0; i<=7 ;i++) {
						small_marker_array[i] = feedback_data->small_marker_corners[i];
					}
					small_marker_rx = feedback_data->small_marker_detected;
					big_marker_rx = feedback_data->big_marker_detected;
					rx_avgx = feedback_data -> x_avg;
					rx_avgy = feedback_data -> y_avg;
					//screen_height = feedback_data -> frame_height;
					//screen_width = feedback_data -> frame_width;
					app_heart_beat = feedback_data -> heartbeat;
					rx_heartbeat = app_heart_beat;
					app_heart_beat = 0;
				});
		vehiclestatus_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/vehicle_status/out", 10,
				[this](const px4_msgs::msg::VehicleStatus::UniquePtr navstatmsg) {
					navstate = navstatmsg->nav_state;
				});
		distancesensor_sub_ =
			this->create_subscription<px4_msgs::msg::DistanceSensor>("fmu/distance_sensor/out", 10,
				[this](const px4_msgs::msg::DistanceSensor::UniquePtr sensordistmsg) {
					altitude_agl = sensordistmsg->current_distance;
					distance_quality = sensordistmsg->signal_quality;
				});

		// Main callback of the node:
		auto timer_callback = [this]()->void {
			
			//Debug print of the received deviation
			// cout << rx_devx << "  " << rx_devy << endl;
			cv::Mat image_copy;
			image_copy2.copyTo(image_copy);
			// Marker circles:
			int thickness = 1; //thickens of the line
			int radius = 5; //Declaring the radius
			cv::Scalar origin_marker_Color(255, 255, 255); // blanco
			cv::Scalar small_marker_Color(0, 0, 255);	//rojo
			cv::Scalar big_marker_Color(0, 255, 0); //verde

			if (small_marker_rx) {
				Point small_tl(small_marker_array[0], small_marker_array[1]); // Top Left, small ref. red square
				Point small_tr(small_marker_array[2], small_marker_array[3]); // Clockwise or top right
				Point small_br(small_marker_array[4], small_marker_array[5]); // Clockwise or bottom right
				Point small_bl(small_marker_array[6], small_marker_array[7]); // Clockwise or bottom left
				
				cv::circle(image_copy, small_tl, radius, origin_marker_Color, thickness);
				cv::circle(image_copy, small_tr, radius, small_marker_Color, thickness);
				cv::circle(image_copy, small_br, radius, small_marker_Color, thickness);
				cv::circle(image_copy, small_bl, radius, small_marker_Color, thickness);

				line(image_copy,small_tl,small_tr,small_marker_Color,1);
				line(image_copy,small_tr,small_br,small_marker_Color,1);
				line(image_copy,small_br,small_bl,small_marker_Color,1);
				line(image_copy,small_bl,small_tl,small_marker_Color,1);
			};
			if (big_marker_rx) {
				Point big_tl(big_marker_array[0], big_marker_array[1]); // Top Left, small ref. red square
				Point big_tr(big_marker_array[2], big_marker_array[3]); // Clockwise or top right
				Point big_br(big_marker_array[4], big_marker_array[5]); // Clockwise or bottom right
				Point big_bl(big_marker_array[6], big_marker_array[7]); // Clockwise or bottom left
				
				line(image_copy,big_tl,big_tr, big_marker_Color,1);
				line(image_copy,big_tr,big_br, big_marker_Color,1);
				line(image_copy,big_br,big_bl, big_marker_Color,1);
				line(image_copy,big_bl,big_tl, big_marker_Color,1);

				cv::circle(image_copy, big_tl, radius, origin_marker_Color, thickness);
				cv::circle(image_copy, big_tr, radius, big_marker_Color, thickness);
				cv::circle(image_copy, big_br, radius, big_marker_Color, thickness);
				cv::circle(image_copy, big_bl, radius, big_marker_Color, thickness);
			};

			// =======  Static Lines overlay  =========== //
			Scalar static_hline_Color(0, 0, 255);
			// Horizontal line // 
			Point static_hpt1(0, (screen_height / 2));
			Point static_hpt2(screen_width , (screen_height / 2));
			line(image_copy,static_hpt1,static_hpt2,static_hline_Color,1);
			// Vertical line // 
			Point static_vpt1((screen_width / 2), 0);
			Point static_vpt2((screen_width / 2), screen_height);
			line(image_copy,static_vpt1,static_vpt2,static_hline_Color,1);

			// =======  Static Text overlay  =========== //

			Point navmode_text_position(20, 20);//Declaring the text position//
			Point altitude_text_position(20, 60);//Declaring the text position//

			double font_size_big = 0.5;//Declaring the font size//
			Scalar font_Color_static(0, 255, 0);//Declaring the color of the font//
			int font_weight = 1;//Declaring the font weight//

			std::string str7 = "Altitude: ";
			std::string str8 = std::to_string(altitude_agl);
			std::string overlaytext_altitude = str7 + str8;
			putText(image_copy, overlaytext_altitude, altitude_text_position,FONT_HERSHEY_COMPLEX, font_size_big,font_Color_static, font_weight);
			
			std::string str5 = "Nav mode: ";
			std::string str6 = std::to_string(navstate);
			std::string overlaytext_navmode = str5 + str6;
			putText(image_copy, overlaytext_navmode, navmode_text_position,FONT_HERSHEY_COMPLEX, font_size_big,font_Color_static, font_weight);

			if (big_marker_rx || small_marker_rx) {
			// =======  Dynamic Lines overlay  =========== //

			Scalar hline_Color(0, 255, 0);
			Point hpt1(0, rx_avgy);
			Point hpt2(screen_width, rx_avgy);
			line(image_copy,hpt1,hpt2,hline_Color,1);

			Point vpt1(rx_avgx, 0);
			Point vpt2(rx_avgx, screen_height);
			line(image_copy,vpt1,vpt2,hline_Color,1);

			// =======  Dynamic Text overlay  =========== //

			double font_size = 0.5;//Declaring the font size//
			Scalar font_Color(0, 255, 0);//Declaring the color of the font//
			int font_weight_small = 1;//Declaring the font weight//
			Point text_devx_position((rx_avgx + 10), 40 );//Declaring the text position//
			Point text_devy_position(40, (rx_avgy - 10));//Declaring the text position//
			std::string str1 = " Deviation X: ";
			std::string str2 = std::to_string(rx_devx);
			std::string str3 = " Deviation Y: ";
			std::string str4 = std::to_string(rx_devy);
			std::string overlaytext_devx = str1 + str2;
			std::string overlaytext_devy = str3 + str4;
			putText(image_copy, overlaytext_devx, text_devx_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight_small);
			putText(image_copy, overlaytext_devy, text_devy_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight_small);

			// ============================================ //
			};
			cv::imshow("Guiado de aterrizaje", image_copy); 
			cv::waitKey(5);
			//image_copy = image_copy2; //flush
		};
	// Main callback function of the node:
	timer_ = this->create_wall_timer(16ms, timer_callback);
	}
private:
	
	float rx_avgx, rx_avgy;
	int small_marker_array [8];
	int big_marker_array [8];
	//int navstate;

	rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<more_interfaces::msg::Devfeedback>::SharedPtr feedback_sub_;
	rclcpp::Subscription<px4_msgs::msg::IrlockReport>::SharedPtr irlock_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehiclestatus_sub_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distancesensor_sub_;  
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};


int main(int argc, char* argv[])
{
	//image_copy2 = image_black;
	image_black.copyTo(image_copy2);
    std::cout << "Program started!" << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundAruco>());

    rclcpp::shutdown();
    return 0;
}