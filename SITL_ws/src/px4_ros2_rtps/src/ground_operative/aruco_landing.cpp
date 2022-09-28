#/*
File: sitl_landing_test.cpp
Author: Manuel Tolino Contreras
Description: Este programa permite al UAV aterrizar sobre el marcador con id Determinada usando IRLOCKREPORT. (Aproximacion bidimensional)
Se utiliza sensor de distancia para tener feedback de la altitud y seleccionar así segundo marcador mas pequeño al estar mas bajo.
*/

#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <string>
#include <algorithm>
#include <iomanip>
#include <sstream>


#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <more_interfaces/msg/devfeedback.hpp>

#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/* Si se va a tratar de usar ROS 2 compressed image transport
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/rgb_colors.h>
#include <sensor_msgs/image_encodings.hpp>
#include "image_transport/image_transport.hpp"
*/


using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

using std::placeholders::_1;

		struct cam_params
	{
		int res_horizontal;
		int res_vertical;
		float fov_horiz;
		float fov_vert;
	};

		struct screen_dev
	{
		double x_avg;
		double y_avg;
		double x_dev;
		double y_dev;
	};
		struct corners_dataset
	{
		int corners_pairs [8];
	};
/**************** OpenCV global variables *****************/
//cv_bridge::CvImage::toImageMsg()
 /* Video object */
 //cv::VideoCapture in_video; rpicamera

//cv::VideoCapture in_video;

//pipeline parameters
int device = 0;                 //0=raspicam 1=usb webcam (when both are connected)
int capture_width = 640 ;
int capture_height = 480 ;
int framerate = 30 ;
int display_width = 640 ;
int display_height = 480 ;

int bitrate = 1024;

std::string GetCurrentTimeForFileName()
{
    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');
    return s;
}
auto videofilename = GetCurrentTimeForFileName();

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
            " height=(int)" + std::to_string(display_height) + " ! queue ! appsink";
}

std::string pipeline = gstreamer_pipeline(device,
                                          capture_width, capture_height, framerate,
                                          display_width, display_height);

std::string gstreamer_writerpipeline(int bitrate, std::string videofilename) {
    return
	// videorate ! video/x-raw, width=640, height=380, framerate=30/1 !
	//" queue ! x264enc speed-preset=ultrafast tune=zerolatency "
            " appsrc ! videorate ! videoconvert !"
            " queue ! x264enc speed-preset=superfast tune=zerolatency"
            " bitrate=" + std::to_string(bitrate) + " !"
            " queue ! matroskamux !"
            " queue ! filesink location=/home/pi/tfm/rpi_foxy_ws/src/rpi_px4_test/src/operative"
            "/" + videofilename + ".mkv"
            " sync=true";
}

std::string writerpipeline = gstreamer_writerpipeline(bitrate, videofilename);

cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
//cv::VideoWriter writer;

/**************** OpenCV parameters *****************/
cv::Mat image /*,image_copy*/;
cv::Mat image_copy;
//cv::String videoInput = "0";

/* ArUco Dictionary ID*/
cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(16));
	/*namespace {
	const char* about = "Detect ArUco marker images";
	const char* keys  =
		"{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
		"DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
		"DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
		"DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
		"DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{h        |false | Print help }"
		"{v        |<none>| Custom video source, otherwise '0' }"
		;
	}*/


/* Camera intrinsic matrix */
const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 474.250810,  0.0,         403.777430,
                                  0.0,       474.152947,    399.072316,
                                  0.0,       0.0,                  1.0);  

/* Distortion*/
const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
const cv::Mat  arucodistCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0); // ToDelete?

/*Other parameters not necessary for now*/
//in_video.open(source); // id

/************           ************/

/************ ROS2 Node ************/

auto irlock_data = px4_msgs::msg::IrlockReport();
auto irlock_msg = px4_msgs::msg::IrlockReport();
auto arucofb_data = more_interfaces::msg::Devfeedback();
auto arucofb_msg = more_interfaces::msg::Devfeedback();

int navstate = -1;
int LANDING_MARKER_BIG = 4;
int LANDING_MARKER_SMALL = 6;
float SWITCH_AGL_ALT = 4;
float SWITCH_AGL_MARGIN = 1.5 ;
float altitude_agl;
int distance_quality;

class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("marker_landing_guidance") {

#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10);
		arucofb_publisher_ = this->create_publisher<more_interfaces::msg::Devfeedback>("aruco_feedback");
		//image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);

#else
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in");
		arucofb_publisher_ = this->create_publisher<more_interfaces::msg::Devfeedback>("aruco_feedback");
		//image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed");

#endif
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
        /*image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&DebugVectAdvertiser::imageCallback, this, _1));*/

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
			/* Some default parameters that should be set to zero each loop */
            int BIG_MARKER_FLAG = 0; // No big marker detected by default
			int SMALL_MARKER_FLAG = 0; // No small marker detected by default
			int DEVIATION_BAD = 0; // This is use to judge the case in the detector function

			
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
        	std::vector<int> ids_valid_big, ids_valid_small;
        	std::vector<std::vector<cv::Point2f>> corners_valid_big, corners_valid_small;
			std::vector<cv::Vec3d> rvecs, tvecs;
			//cv::Mat image_copy(1200, 1200, CV_8UC3, cv::Scalar(0, 0, 0));
			
			//float fov_horiz = 1.570796327 ;
			//float fov_vert = 1.570796327 ;
			camera_parameters.fov_horiz = 1.570796327;
			camera_parameters.fov_vert = 1.570796327;

			double x_avg, y_avg, x_dev, y_dev;

			//in_video.grab();
			//in_video.retrieve(image);
			//std::cout<<"About to read"<<std::endl;
			
			if (!cap.read(image)) {
            std::cout<<"Capture read error"<<std::endl;
            //break;
        	}
			image.copyTo(image_copy);

			camera_parameters.res_horizontal = image_copy.size().width;
			camera_parameters.res_vertical = image_copy.size().height;
			//camera_parameters.res_horizontal = 640;
			//camera_parameters.res_vertical = 480;

			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
			/*
			int TRACKBAR_AGL_INPUT;
			int TRACKBAR_AGL_MARGIN;

			createTrackbar("Transition altitude (dm):", "Detected markers", &TRACKBAR_AGL_INPUT, 70);
			createTrackbar("Altitude margin (dm):", "Detected markers", &TRACKBAR_AGL_MARGIN, 30);

			if (TRACKBAR_AGL_INPUT <= TRACKBAR_AGL_MARGIN) {
				TRACKBAR_AGL_INPUT = TRACKBAR_AGL_MARGIN + 1;
			}

			SWITCH_AGL_ALT = TRACKBAR_AGL_INPUT / 10;
			SWITCH_AGL_MARGIN = TRACKBAR_AGL_INPUT / 10;
			*/
			SWITCH_AGL_ALT = 1.5;
			SWITCH_AGL_MARGIN = 1.0;

			//if (navstate == 20 /*&& in_video.isOpened()*/){
				/* MOVED OUTSIDE FOR DEBUGGING
				in_video.grab();
				in_video.retrieve(image);
				image.copyTo(image_copy);
				cv::aruco::detectMarkers(image, dictionary, corners, ids);
				cv::aruco::drawDetectedMarkers(image_copy, corners, ids);
				//imshow("cam", image_copy); 
				*/

			// Reset all the detected markers to the new incoming calculated data
				ids_valid_big = ids;
        		corners_valid_big = corners;
				ids_valid_small = ids;
        		corners_valid_small = corners;
			//};

            //int res_horizontal = image_copy.size().width;
            //int res_vertical = image_copy.size().height;

            /*  DEBUGGING PRINT of camera input total pixels */
            // printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);

            //if (ids.size() > 0 && navstate == 20){ // If at least one marker detected and Prec Land mode

			if (ids.size() > 0){ // If at least one marker detected

				for (int i = 0; i < int(ids.size()); i++){
					if (int(ids.at(i)) == LANDING_MARKER_BIG){
						BIG_MARKER_FLAG = 1;
						ids_valid_big.resize(1);
						corners_valid_big.resize(1);
						ids_valid_big[0]=ids.at(i);
						corners_valid_big[0]=corners.at(i);
						//cv::aruco::estimatePoseSingleMarkers(corners_valid, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
					};
					if (int(ids.at(i)) == LANDING_MARKER_SMALL){
						SMALL_MARKER_FLAG = 1;
						ids_valid_small.resize(1);
						corners_valid_small.resize(1);
						ids_valid_small[0]=ids.at(i);
						corners_valid_small[0]=corners.at(i);
						//cv::aruco::estimatePoseSingleMarkers(corners_valid, 0.05, intrinsic_matrix, distCoeffs, rvecs, tvecs);
					}
				}

				screen_dev deviation, deviation_big, deviation_small;
				corners_dataset small_marker_pairs, big_marker_pairs;
				//cam_params camera_parameters;

				x_avg = 0;
				y_avg = 0;
				x_dev = 0;
				y_dev = 0;

				if ((altitude_agl < SWITCH_AGL_ALT) /*&& (BIG_MARKER_FLAG == 1 && SMALL_MARKER_FLAG == 1)*/) 
				//If I'm below transition altitude and I detect Both markers
				
				{
					//Tell the ground station this...
					arucofb_data.small_marker_detected = true;
					arucofb_data.big_marker_detected = true;
					//Calculate deviation for both markers...
					calcDev(corners_valid_big,0,camera_parameters,&deviation_big, &big_marker_pairs);
					calcDev(corners_valid_small,0,camera_parameters,&deviation_small, &small_marker_pairs);

					for (int i=0; i<=7 ;i++) { //Send corners for screen representation in ground station
						arucofb_data.big_marker_corners[i] = big_marker_pairs.corners_pairs[i];
						arucofb_data.small_marker_corners[i] = small_marker_pairs.corners_pairs[i];
					};
					if (altitude_agl >= (SWITCH_AGL_ALT - SWITCH_AGL_MARGIN) && altitude_agl <= SWITCH_AGL_ALT) {

						float ratio = (altitude_agl - (SWITCH_AGL_ALT - SWITCH_AGL_MARGIN)) / SWITCH_AGL_MARGIN;

						x_dev = ratio * deviation_big.x_dev + (1 - ratio)* deviation_small.x_dev;
						y_dev = ratio * deviation_big.y_dev + (1 - ratio)* deviation_small.y_dev;
						x_avg = ratio * deviation_big.x_avg + (1 - ratio)* deviation_small.x_avg;
						y_avg = ratio * deviation_big.y_avg + (1 - ratio)* deviation_small.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
					} else if (altitude_agl < (SWITCH_AGL_ALT-SWITCH_AGL_MARGIN)) { 
						calcDev(corners_valid_small,0,camera_parameters,&deviation,&small_marker_pairs);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
					} else if (altitude_agl > (SWITCH_AGL_ALT-SWITCH_AGL_MARGIN)) {
						calcDev(corners_valid_big,0,camera_parameters,&deviation,&big_marker_pairs);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
					}
				} else if ((altitude_agl < SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 1 && SMALL_MARKER_FLAG == 0)){
				// But if I only detect the big marker below the transition altitude, only take readings from it
					//Tell the ground station this...
					arucofb_data.small_marker_detected = false;
					arucofb_data.big_marker_detected = true;
						calcDev(corners_valid_big,0,camera_parameters,&deviation,&big_marker_pairs);
						for (int i=0; i<=7 ;i++) { //Send corners for screen representation in ground station
							arucofb_data.big_marker_corners[i] = big_marker_pairs.corners_pairs[i];
							arucofb_data.small_marker_corners[i] = 0;
						};
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
				} else if ((altitude_agl < SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 0 && SMALL_MARKER_FLAG == 1)){
				// Also if I only detect the small marker below the transition altitude, only take readings from it
						arucofb_data.small_marker_detected = true;
						arucofb_data.big_marker_detected = false;
						calcDev(corners_valid_small,0,camera_parameters,&deviation,&small_marker_pairs);
						for (int i=0; i<=7 ;i++) { //Send corners for screen representation in ground station
							arucofb_data.big_marker_corners[i] = 0;
							arucofb_data.small_marker_corners[i] = small_marker_pairs.corners_pairs[i];
						};
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
				} else if ((altitude_agl > SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 1)){
				// If I'm higher than the transition altitude and I have big marker on sight...
						arucofb_data.small_marker_detected = false;
						arucofb_data.big_marker_detected = true;
						calcDev(corners_valid_big,0,camera_parameters,&deviation,&small_marker_pairs);
						for (int i=0; i<=7 ;i++) { //Send corners for screen representation in ground station
							arucofb_data.big_marker_corners[i] = big_marker_pairs.corners_pairs[i];
							arucofb_data.small_marker_corners[i] = 0;
						};
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						arucofb_data.x_avg = x_avg;
						arucofb_data.y_avg = y_avg;
						//cout << x_dev << "  " << y_dev << endl;
				} else {
						arucofb_data.small_marker_detected = false;
						arucofb_data.big_marker_detected = false;
						for (int i=0; i<=7 ;i++) { //Send corners for screen representation in ground station
							arucofb_data.big_marker_corners[i] = 0;
							arucofb_data.small_marker_corners[i] = 0;
						};
					// In any other case no data should be published to the autopilot
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = NAN;
                		irlock_data.pos_y = NAN;
						arucofb_data.x_avg = 0;
						arucofb_data.y_avg = 0;
						DEVIATION_BAD = 0; // Do not represent the target indicator on screen
						//cout << "Bad readings..." << endl;
				}

				if (DEVIATION_BAD == 0) {

					// =======  Dynamic Lines overlay  =========== //

					Scalar hline_Color(0, 255, 0);
					Point hpt1(0, y_avg);
					Point hpt2(camera_parameters.res_horizontal, y_avg);
					line(image_copy,hpt1,hpt2,hline_Color,1);

					Point vpt1(x_avg, 0);
					Point vpt2(x_avg, camera_parameters.res_vertical);
					line(image_copy,vpt1,vpt2,hline_Color,1);

					// =======  Dynamic Text overlay  =========== //

					double font_size = 0.5;//Declaring the font size//
					Scalar font_Color(0, 255, 0);//Declaring the color of the font//
					int font_weight_small = 1;//Declaring the font weight//
					Point text_devx_position((x_avg + 10), 40 );//Declaring the text position//
					Point text_devy_position(40, (y_avg - 10));//Declaring the text position//
					std::string str1 = " Deviation X: ";
					std::string str2 = std::to_string(irlock_msg.pos_x);
					std::string str3 = " Deviation Y: ";
					std::string str4 = std::to_string(irlock_msg.pos_y);
					std::string overlaytext_devx = str1 + str2;
					std::string overlaytext_devy = str3 + str4;
					putText(image_copy, overlaytext_devx, text_devx_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight_small);
					putText(image_copy, overlaytext_devy, text_devy_position,FONT_HERSHEY_SIMPLEX, font_size,font_Color, font_weight_small);

					// ============================================ //
				}
			

				// Prepare the rest of the message for the autopilot:

				irlock_data.timestamp = timestamp_.load();
                irlock_data.signature = 1;
                irlock_data.size_x = 0;
                irlock_data.size_y = 0;
				irlock_msg = irlock_data;

				arucofb_data.frame_height = capture_height;
				arucofb_data.frame_width = capture_width;
				arucofb_msg = arucofb_data;
				// Publish the message to the autopilot:

				this->publisher_->publish(irlock_msg);
				this->arucofb_publisher_->publish(arucofb_msg);

			};

				// =======  Static Lines overlay  =========== //

				// Horizontal line // 
				Scalar static_hline_Color(0, 0, 255);
				Point static_hpt1(0, (camera_parameters.res_vertical / 2));
				Point static_hpt2(camera_parameters.res_horizontal , (camera_parameters.res_vertical / 2));
				line(image_copy,static_hpt1,static_hpt2,static_hline_Color,1);

				// Vertical line // 
				Point static_vpt1((camera_parameters.res_horizontal / 2), 0);
				Point static_vpt2((camera_parameters.res_horizontal / 2), camera_parameters.res_vertical);
				line(image_copy,static_vpt1,static_vpt2,static_hline_Color,1);

				// =======  Static Text overlay  =========== //

				Point text2_position(20, 20);//Declaring the text position//
				Point text3_position(20, 60);//Declaring the text position//

				double font_size_big = 0.5;//Declaring the font size//
				Scalar font_Color_static(0, 255, 0);//Declaring the color of the font//
				int font_weight = 1;//Declaring the font weight//

				std::string str7 = "Altitude: ";
				std::string str8 = std::to_string(altitude_agl);
				std::string overlaytext_altitude = str7 + str8;
				putText(image_copy, overlaytext_altitude, text3_position,FONT_HERSHEY_COMPLEX, font_size_big,font_Color_static, font_weight);
				
				std::string str5 = "Nav mode: ";
				std::string str6 = std::to_string(navstate);
				std::string overlaytext_navmode = str5 + str6;
				putText(image_copy, overlaytext_navmode, text2_position,FONT_HERSHEY_COMPLEX, font_size_big,font_Color_static, font_weight);
				//videowriter.write(image_copy);
				//writer << image_copy;
                //cv::imshow("Detected markers", image_copy); 
				//cv::waitKey(5);

			//calculate frame rate
			Tend = std::chrono::steady_clock::now();
			f = std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
			Tbegin = Tend;
			if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
			for(f=0.0, i_fps=0;i_fps<16;i_fps++){ f+=FPS[i_fps]; }
			printf("---- Process FPS %0.2f --- Xdev: %0.2f \n", f/16, x_dev);
		};

		auto timer_imagetransmit_callback = [this]()->void {
			//printf("1 s Counter\n");
				/*
				if (!image_copy.empty()) {
				writer << image_copy;
				}
				*/
				//cv::cvtColor(image_copy,image_bgr,cv::COLOR_RGB2BGR);
				//std::cout << "Frame written" << std::endl;
				//cv::waitKey(5);
				int dummy = 1;
				};
		

	// Main callback function of the node:

	timer_ = this->create_wall_timer(33ms, timer_callback);
	timer_imagetransmit_ = this->create_wall_timer(33ms, timer_imagetransmit_callback);

	// ============= Show the result video feed on screen =============== //

	//std_msgs::msg::Header hdr;
	//sensor_msgs::msg::CompressedImage::SharedPtr img_msg;
	//img_msg = cv_bridge::CvImage(hdr, "bgr8", image_copy).toCompressedImageMsg();
	//this->image_pub_->publish(*img_msg);
	}
private:

	//screen_dev deviation;
	//screen_dev deviation_big, deviation_small;
	
	cam_params camera_parameters;
	std::chrono::steady_clock::time_point Tbegin, Tend;
	float f;
	float FPS[16];
	int   i_fps,Fcnt=0;
	/**
	 * @brief Calculate the deviation from center image of a detected marker
	 * @param corners   Detected corners by OpenCV detectMarkers fnc
	 * @param mkr_index    Which marker of all detected in the vector of 'corners' - usually 1st (0)
	 * @param camera_parameters    FOV and image resolution parameters you need to get from video input object
	 */
	void calcDev(std::vector<std::vector<cv::Point2f>> corners, int mkr_index, cam_params camera_parameters, screen_dev* deviation, corners_dataset* corners_coord) {

                //auto selected_marker = corners[marker_index];
				auto selected_marker = corners[mkr_index];
                auto corner1 = selected_marker[0]; // Top Left, small ref. red square
                auto corner2 = selected_marker[1]; // Clockwise or top right
                auto corner3 = selected_marker[2]; // Clockwise or bottom right
                auto corner4 = selected_marker[3]; // Clockwise or bottom left

                double x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
                double y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

				corners_coord->corners_pairs[0] = corner1.x;
				corners_coord->corners_pairs[1] = corner1.y;
				corners_coord->corners_pairs[2] = corner2.x;
				corners_coord->corners_pairs[3] = corner2.y;
				corners_coord->corners_pairs[4] = corner3.x;
				corners_coord->corners_pairs[5] = corner3.y;
				corners_coord->corners_pairs[6] = corner4.x;
				corners_coord->corners_pairs[7] = corner4.y;
				// For monitoring:

                deviation->x_avg = x_sum / 4;
                deviation->y_avg = y_sum / 4;

				deviation->x_dev = (deviation->x_avg - camera_parameters.res_horizontal * .5) * camera_parameters.fov_horiz / camera_parameters.res_horizontal;
                deviation->y_dev = (deviation->y_avg - camera_parameters.res_vertical * .5) * camera_parameters.fov_vert / camera_parameters.res_vertical;
	};

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_imagetransmit_;

	rclcpp::Publisher<px4_msgs::msg::IrlockReport>::SharedPtr publisher_;
	rclcpp::Publisher<more_interfaces::msg::Devfeedback>::SharedPtr arucofb_publisher_;
	//rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehiclestatus_sub_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distancesensor_sub_;   
	//rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
 
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};


int main(int argc, char* argv[])
{
	
	/*

	//writer.open("appsrc ! videoconvert ! video/x-raw,width=640,height=480,framerate=30/1 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.0.12 port=9999 sync=false"
                , 0, (double)30, cv::Size(640, 480), true);

	*/
	
	
	// WRITE TO FILE PIPELINE:
/*
	writer.open( writerpipeline, 0, (double)25, cv::Size(640, 480), true);
	std::cout << "Using writer pipeline: \n\t" << writerpipeline << "\n\n\n";
    if (!writer.isOpened()) {
        printf("=ERR= can't create video writer\n");
        return -1;
    }
	*/
	// UDP STREAM PIPELINE:
/*
	writer.open("appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=512 byte-stream=true threads=1 ! mpegtsmux ! queue ! udpsink host=192.168.0.12 port=5666 sync=false"
                , 0, (double)30, cv::Size(640, 480), true);
	    if (!writer.isOpened()) {
        printf("=ERR= can't create video writer\n");
        return -1;
    }
*/
    std::cout << "Using reader pipeline: \n\t" << pipeline << "\n\n\n";
    if(!cap.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
        return (-1);
    }
	/*if(!videowriter.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
        return (-1);
    }*/

    //cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    //in_video.open(0); //rpicamera
	//in_video.set(cv::CAP_PROP_FPS, 30);
	//in_video.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	//in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 1024);
	//in_video.set(cv::CAP_PROP_SATURATION, 0);
    //in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
	// HITL:
	//in_video.open("-v udpsrc port=5600 caps ='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! fpsdisplaysink sync=false");

	std::cout << "Video input received!" << std::endl;
	std::cout << "Starting ArUco autoland control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    //in_video.release();
    rclcpp::shutdown();
    return 0;
}