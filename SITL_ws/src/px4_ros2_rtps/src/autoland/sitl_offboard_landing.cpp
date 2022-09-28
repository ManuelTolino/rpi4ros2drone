#/*
File: sitl_offboard_landing.cpp
Author: Manuel Tolino Contreras
Description: Este programa permite al UAV aterrizar sobre el marcador con id Determinada usando IRLOCKREPORT. (Aproximacion bidimensional)
Se utiliza sensor de distancia para tener feedback de la altitud y seleccionar asi segundo marcador mas pequeno al estar mas bajo. Incluye una
especie de 'state machine' que gobierna el UAV de forma autonoma. Si la camara no detecta el marcador con ID=5 en mitad de la patruya,
no continuara.

git clone <repository>
 
git reset --hard <COMMIT-SHA-ID>

compatible stack:
git checkout deb938fceaa79b927ae0f622d8b2b8c4ad10d391
git PX4 SHA: 79a34b5aed8ee9eb10afda3ae83f1be02ad897a8
ROS_COM : 042d41ab66f25e161cf1cd449ba06727edfdc566
MSGS: be36a7dc24544495939e1a1d2d85843bebaa763c
*/

#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
//#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
//#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <opencv2/core.hpp> 
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// for compressing the image
//#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/rgb_colors.h>
#include <sensor_msgs/image_encodings.hpp>
//#include <image_transport/image_transport.hpp>
//#include "cv_bridge/cv_bridge.h"
//#include "image_transport/image_transport.hpp"
//#include "compressed_image_transport/compressed_publisher.h"
//#include "compressed_image_transport/compression_common.h"
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <iostream>

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/// @brief Camera parameters
struct cam_params{
	int res_horizontal;
	int res_vertical;
	float fov_horiz;
	float fov_vert;
};
/// @brief Calculated prec landing tangential deviation on screen
struct screen_dev{
	double x_avg;
	double y_avg;
	double x_dev;
	double y_dev;
};

/**************** OpenCV global variables *****************/

//cv_bridge::CvImage::toImageMsg();
 /* Video object */
cv::VideoCapture in_video;

/**************** OpenCV parameters *****************/

cv::String videoInput = "0";

/* ArUco Dictionary ID*/
cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(16)); //Selected ORiginalArUco

/************ ROS2 Node ************/

auto irlock_data = px4_msgs::msg::IrlockReport();
auto irlock_msg = px4_msgs::msg::IrlockReport();

float SWITCH_AGL_ALT = 3.0;
float SWITCH_AGL_MARGIN = 1.0 ;
float altitude_agl;

int navstate = -1;
int LANDING_MARKER_BIG = 4;
int armedstate = 0;
int LANDING_MARKER_SMALL = 6;
int TRACKBAR_AGL_INPUT = 30;
int TRACKBAR_AGL_MARGIN = 10;
int distance_quality;
int first_loop = 1;
int take_off_loop = 0;
int offboard_order = 0;


class SITLOffboardLanding : public rclcpp::Node
{
public:
	SITLOffboardLanding() : Node("marker_landing_guidance") {

#ifdef ROS_DEFAULT_API
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10);
		//image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", 10);
    	//info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
	
#else
		publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in");
		//mage_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed");
    	//info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info");
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("fmu/vehicle_command/in");
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		
#endif
		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});
		vehiclestatus_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleStatus>("fmu/vehicle_status/out", 10,
				[this](const px4_msgs::msg::VehicleStatus::UniquePtr navstatmsg) {
					navstate = navstatmsg->nav_state; // 20 = precland // 14 = offboard
					armedstate = navstatmsg->arming_state; //armed = 2
				});
		distancesensor_sub_ =
			this->create_subscription<px4_msgs::msg::DistanceSensor>("fmu/distance_sensor/out", 10,
				[this](const px4_msgs::msg::DistanceSensor::UniquePtr sensordistmsg) {
					altitude_agl = sensordistmsg->current_distance;
					distance_quality = sensordistmsg->signal_quality;
				});

		// Main callback of the node:
		auto timer_callback = [this]()->void {

			cv::Mat image /*,image_copy*/;
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
        	std::vector<int> ids_valid_big, ids_valid_small;
        	std::vector<std::vector<cv::Point2f>> corners_valid_big, corners_valid_small;
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::Mat image_copy(1200, 1200, CV_8UC3, cv::Scalar(0, 0, 0));

			in_video.grab();
			in_video.retrieve(image);
			image.copyTo(image_copy);

			camera_parameters.fov_horiz = 1.570796327;
			camera_parameters.fov_vert = 1.570796327;
			camera_parameters.res_horizontal = image_copy.size().width;
			camera_parameters.res_vertical = image_copy.size().height;

			cv::aruco::detectMarkers(image, dictionary, corners, ids);
			cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

			// =====  PREC. LAND Monitor trackbar  ===== //

			createTrackbar("Transition altitude (dm):", "Detected markers", &TRACKBAR_AGL_INPUT, 70);
			createTrackbar("Altitude margin (dm):", "Detected markers", &TRACKBAR_AGL_MARGIN, 30);
			if (TRACKBAR_AGL_INPUT <= TRACKBAR_AGL_MARGIN) {
				TRACKBAR_AGL_INPUT = TRACKBAR_AGL_MARGIN + 1;
			}
			SWITCH_AGL_ALT = float (TRACKBAR_AGL_INPUT / 10.0);
			SWITCH_AGL_MARGIN = float (TRACKBAR_AGL_INPUT / 10.0);

			//===========================================//

			// Reset variables for new incoming data
			ids_valid_big = ids;
			corners_valid_big = corners;
			ids_valid_small = ids;
			corners_valid_small = corners;
			BIG_MARKER_FLAG = 0; // No big marker detected by default
			SMALL_MARKER_FLAG = 0; // No small marker detected by default

			if (ids.size() > 0){ // Flag for the state machine
				for (int i = 0; i < int(ids.size()); i++){
					if (int(ids.at(i)) == 5){ 
						TARGET_SURVEY_ONSIGHT = 1;
					}
				}
			}
			// If at least one marker detected and PREC. LAND mode active
            if (ids.size() > 0 && navstate == 20){ 
				for (int i = 0; i < int(ids.size()); i++){
					if (int(ids.at(i)) == LANDING_MARKER_BIG){
						BIG_MARKER_FLAG = 1;
						ids_valid_big.resize(1);
						corners_valid_big.resize(1);
						ids_valid_big[0]=ids.at(i);
						corners_valid_big[0]=corners.at(i);
					};
					if (int(ids.at(i)) == LANDING_MARKER_SMALL){
						SMALL_MARKER_FLAG = 1;
						ids_valid_small.resize(1);
						corners_valid_small.resize(1);
						ids_valid_small[0]=ids.at(i);
						corners_valid_small[0]=corners.at(i);
					}
				}

				screen_dev deviation, deviation_big, deviation_small;

				// ========  Main precission landing algorithm:   ========= //

				if ((altitude_agl < SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 1 && SMALL_MARKER_FLAG == 1)) 
				//If I'm below transition altitude and I detect Both markers
				{
					calcDev(corners_valid_big,0,camera_parameters,&deviation_big);
					calcDev(corners_valid_small,0,camera_parameters,&deviation_small);

					if (altitude_agl >= (SWITCH_AGL_ALT - SWITCH_AGL_MARGIN) && altitude_agl <= SWITCH_AGL_ALT) {

						float ratio = (altitude_agl - (SWITCH_AGL_ALT - SWITCH_AGL_MARGIN)) / SWITCH_AGL_MARGIN;

						x_dev = ratio * deviation_big.x_dev + (1 - ratio)* deviation_small.x_dev;
						y_dev = ratio * deviation_big.y_dev + (1 - ratio)* deviation_small.y_dev;
						x_avg = ratio * deviation_big.x_avg + (1 - ratio)* deviation_small.x_avg;
						y_avg = ratio * deviation_big.y_avg + (1 - ratio)* deviation_small.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
					} else if (altitude_agl < (SWITCH_AGL_ALT-SWITCH_AGL_MARGIN)) { 
						calcDev(corners_valid_small,0,camera_parameters,&deviation);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
					} else if (altitude_agl > (SWITCH_AGL_ALT-SWITCH_AGL_MARGIN)) {
						calcDev(corners_valid_big,0,camera_parameters,&deviation);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
					}
				} else if ((altitude_agl < SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 1 && SMALL_MARKER_FLAG == 0)){
				// But if I only detect the big marker below the transition altitude, only take readings from it
						calcDev(corners_valid_big,0,camera_parameters,&deviation);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
				} else if ((altitude_agl < SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 0 && SMALL_MARKER_FLAG == 1)){
				// Also if I only detect the small marker below the transition altitude, only take readings from it
						calcDev(corners_valid_small,0,camera_parameters,&deviation);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
				} else if ((altitude_agl > SWITCH_AGL_ALT) && (BIG_MARKER_FLAG == 1)){
				// If I'm higher than the transition altitude and I have big marker on sight...
						calcDev(corners_valid_big,0,camera_parameters,&deviation);
						x_dev = deviation.x_dev;
						y_dev = deviation.y_dev;
						x_avg = deviation.x_avg;
						y_avg = deviation.y_avg;
						irlock_data.pos_x = x_dev;
                		irlock_data.pos_y = y_dev;
						DEVIATION_BAD = 0; 
						//cout << x_dev << "  " << y_dev << endl;
				} else {
					// In any other case no data should be published to the autopilot
						x_dev = -9999;
						y_dev = -9999;
						x_avg = -9999;
						y_avg = -9999;
						irlock_data.pos_x = NAN;
                		irlock_data.pos_y = NAN;
						DEVIATION_BAD = 1; // Do not represent the target indicator on screen
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
					putText(image_copy, overlaytext_devx, text_devx_position,FONT_HERSHEY_SIMPLEX, 
					font_size,font_Color, font_weight_small);
					putText(image_copy, overlaytext_devy, text_devy_position,FONT_HERSHEY_SIMPLEX, 
					font_size,font_Color, font_weight_small);

					// ============================================ //
				}

				// Prepare the rest of the message for the autopilot:

				irlock_data.timestamp = timestamp_.load();
                irlock_data.signature = 1;
                irlock_data.size_x = 0;
                irlock_data.size_y = 0;
				irlock_msg = irlock_data;

				// Publish the message to the autopilot:

				this->publisher_->publish(irlock_msg);
			};
				
				// =======  Static Lines overlay  =========== //

				Scalar static_hline_Color(0, 0, 255);
				Point static_hpt1(0, (camera_parameters.res_horizontal / 2));
				Point static_hpt2(camera_parameters.res_horizontal , (camera_parameters.res_horizontal / 2));
				line(image_copy,static_hpt1,static_hpt2,static_hline_Color,1);

				Point static_vpt1((camera_parameters.res_vertical / 2), 0);
				Point static_vpt2((camera_parameters.res_vertical / 2), camera_parameters.res_vertical);
				line(image_copy,static_vpt1,static_vpt2,static_hline_Color,1);

				// =======  Static Text overlay  =========== //

				Point text2_position(40, 40);//Declaring the text position//
				Point text3_position(40, 80);//Declaring the text position//

				double font_size_big = 0.7;//Declaring the font size//
				Scalar font_Color_static(0, 255, 0);//Declaring the color of the font//
				int font_weight = 2;//Declaring the font weight//

				std::string str7 = "Altitude: ";
				std::string str8 = std::to_string(altitude_agl);
				std::string overlaytext_altitude = str7 + str8;
				putText(image_copy, overlaytext_altitude, text3_position,FONT_HERSHEY_COMPLEX, 
				font_size_big,font_Color_static, font_weight);
				
				std::string str5 = "Nav mode: ";
				std::string str6 = std::to_string(navstate);
				std::string overlaytext_navmode = str5 + str6;
				putText(image_copy, overlaytext_navmode, text2_position,FONT_HERSHEY_COMPLEX, 
				font_size_big,font_Color_static, font_weight);

				// ============= Show the result video feed on screen =============== //

                cv::imshow("Detected markers", image_copy); 
				cv::waitKey(2);	
				// ============= Transmit the image via ROS2 compressed image topic =============== //
				//std_msgs::msg::Header hdr;
				//sensor_msgs::msg::Image::SharedPtr msg;

				//msg = cv_bridge::CvImage(hdr, "bgr8", image_copy).toImageMsg();
				//sensor_msgs::msg::CompressedImage::SharedPtr img_msg;
				//img_msg = cv_bridge::CvImage(hdr, "bgr8", image_copy).toCompressedImageMsg();
				//this->image_pub_->publish(*img_msg);
		};
		auto timer_status_callback = [this]()->void {
			if (first_loop == 1){
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this -> arm();
				cout << "Orden de armado enviada" << endl;
				first_loop = 0;
			};
			if (take_off_loop == 3){
				OFF_X = 0.0 ;
				OFF_Y = 0.0 ;
				OFF_YAW = -3.14 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
			};
			if (take_off_loop == 5){
				OFF_X = 0.0 ;
				OFF_Y = 0.0 ;
				OFF_YAW = 0.0 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
			};
			if (take_off_loop == 10){
			 	OFF_X = 4.0 ;
				OFF_Y = 0.0 ;
				OFF_YAW = 0.0 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
			};
			if (take_off_loop == 20){
				OFF_X = 3.0 ;
			 	OFF_Y = 4.0 ;
				OFF_YAW = 0.0 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
			};
			if (take_off_loop == 30){
				SURVEY_OK = TARGET_SURVEY_ONSIGHT;
				OFF_X = 0.0 ;
				OFF_Y = 3.0 ;
				OFF_YAW = 0.0 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
				cout << "Necesito objetivo ID=5 la vista para continuar" << endl;
			};
			if (take_off_loop == 40){
				OFF_X = 0.0 ;
				OFF_Y = -1.0 ;
				OFF_YAW = 0.0 ;
				cout << "Go To X = " << OFF_X << " Y = " << OFF_Y << " Z = " << SWITCH_AGL_ALT << " PSI = " << OFF_YAW << endl;
			};
			if (take_off_loop == 50){
				this->publish_precland_mode(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 4, 9);
				cout << "Iniciando secuencia de aterrizaje de precision" << endl;
			};
			if (take_off_loop < 51) {
				int SURVEY_FLAG = 0;
				int ARMED_FLAG = 0;
				cout << "TO Loop Debug value: " << take_off_loop << endl;
				if (!((take_off_loop == 30) && !SURVEY_OK)){
					SURVEY_FLAG = 1;
				}
				/*if (!((take_off_loop == 2) & !(armedstate == 2))){
					ARMED_FLAG = 1;
				}*/
				if(SURVEY_FLAG){
					take_off_loop++;
				}
			};
		};
		auto timer_offboard_callback = [this]()->void {
				// Both topics below should be published together
				publish_offboard_control_mode();
				TrajectorySetpoint msg{};
				msg.timestamp = timestamp_.load();
				msg.position = {OFF_X, OFF_Y, -SWITCH_AGL_ALT};
				msg.yaw = OFF_YAW; // [-PI:PI]

				trajectory_setpoint_publisher_->publish(msg);
		};
	// Main thread, marker detection and guided land. Desired 30 FPS
	timer_ = this->create_wall_timer(16ms, timer_callback);
	// Trajectory set-point thread
	timer_offboard_ = this->create_wall_timer(100ms, timer_offboard_callback);
	// State machine
	timer_satus_ = this->create_wall_timer(1000ms, timer_status_callback);
	}
	void arm() const;
private:

	double x_avg = 0;
	double y_avg = 0;
	double x_dev = 0;
	double y_dev = 0;
	/* Some default parameters that should be set to zero each loop */
	int BIG_MARKER_FLAG = 0; // No big marker detected by default
	int SMALL_MARKER_FLAG = 0; // No small marker detected by default
	int DEVIATION_BAD = 0; // This is use to judge the case in the detector function
	float OFF_X = 0.0 ;
	float OFF_Y = 0.0 ;
	float OFF_YAW = 0.0 ;
	int TARGET_SURVEY_ONSIGHT = 0;
	int SURVEY_OK = 0;

	cam_params camera_parameters;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void publish_precland_mode(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0, float param3 = 0.0) const;
	void publish_offboard_control_mode() const;
	//void publish_trajectory_setpoint() const;
		
	/**
	 * @brief Calculate the deviation from center image of a detected marker
	 * @param corners   Detected corners by OpenCV detectMarkers fnc
	 * @param mkr_index    Which marker of all detected in the vector of 'corners' - usually 1st (0)
	 * @param camera_parameters    FOV and image resolution parameters you need to get from video input object
	 */
	void calcDev(std::vector<std::vector<cv::Point2f>> corners, int mkr_index, 
	cam_params camera_parameters, screen_dev* deviation) {

                //auto selected_marker = corners[marker_index];
				auto selected_marker = corners[mkr_index];
                auto corner1 = selected_marker[0]; // Top Left, small ref. red square
                auto corner2 = selected_marker[1]; // Clockwise or top right
                auto corner3 = selected_marker[2]; // Clockwise or bottom right
                auto corner4 = selected_marker[3]; // Clockwise or bottom left

                double x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
                double y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

                deviation->x_avg = x_sum / 4;
                deviation->y_avg = y_sum / 4;

				deviation->x_dev = (deviation->x_avg - camera_parameters.res_horizontal * .5) 
				* camera_parameters.fov_horiz / camera_parameters.res_horizontal;
                deviation->y_dev = (deviation->y_avg - camera_parameters.res_vertical * .5) 
				* camera_parameters.fov_vert / camera_parameters.res_vertical;
	};

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_satus_;
	rclcpp::TimerBase::SharedPtr timer_offboard_;

	rclcpp::Publisher<px4_msgs::msg::IrlockReport>::SharedPtr publisher_;
	//rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
	//rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehiclestatus_sub_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distancesensor_sub_;    
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
};

/**
 * @brief Send a command to Arm the vehicle
 */
void SITLOffboardLanding::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void SITLOffboardLanding::publish_offboard_control_mode() const {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */

void SITLOffboardLanding::publish_precland_mode(uint16_t command, float param1,
					      float param2, float param3) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void SITLOffboardLanding::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    // in_video.open(0); rpicamera
	// Gstreamer blockling input pipeline. Node wont start if UDP video input not received
    in_video.open("udpsrc port=5601 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
	std::cout << "Video input received!" << std::endl;
	std::cout << "Starting ArUco autoland control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SITLOffboardLanding>());

    in_video.release();
    rclcpp::shutdown();
    return 0;
}