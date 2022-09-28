#/*
File: sitl_landing_test.cpp
Author: Manuel Tolino Contreras
Description: En este programa se intenta enviar a PX4 información de posición relativa a marcador para
ser integrada en el EKF y obtener información local de posición sin GPS.

For debugging odometry:

ros2 topic echo /fmu/vehicle_visual_odometry/in
ros2 topic echo /fmu/vehicle_odometry/out


EKF2_AID_MASK bitmask parameter:
(convert binary to decimal e.g. 110000000)
0: use GPS
1: use optical flow
2: inhibit IMU bias estimation
3: vision position fusion
4: vision yaw fusion
5: multi-rotor drag fusion
6: rotate external vision
7: GPS yaw fusion
8: vision velocity fusion

MPC_JERK_AUTO 0.5 > 500.0 (1) default is 8.0 m/s^3

param set-default LTEST_MODE 1
param set-default PLD_HACC_RAD 0.1
param set-default MAV_ODOM_LP 1
param set-default RTL_PLD_MD 2
param set-default MPC_LAND_SPEED 0.5
param set-default EKF2_AID_MASK 10
param set-default EKF2_HGT_MODE 2
param set-default MPC_JERK_AUTO 3.0





*/

#include <cstdlib>

#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_visual_odometry.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
//#include <opencv2/core/eigen.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

#include <chrono>
#include <iostream>
#include <iomanip>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

/**************** OpenCV global variables *****************/

 /* Video object */
 //cv::VideoCapture in_video; rpicamera
cv::VideoCapture in_video;

/**************** OpenCV parameters *****************/

cv::String videoInput = "0";

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
float fov_horiz = 1.0 ;
float fov_vert = 1.0 ;

/* Camera intrinsic matrix 
const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 530.8269276712998,  0.0,       320.5,
                                  0.0,       530.8269276712998,  240.5,
                                  0.0,       0.0,                  1.0); */
const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 600,  0.0,         600,
                                  0.0,       600,    600,
                                  0.0,       0.0,                  1.0);   

/* Distortion*/

const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
const cv::Mat  arucodistCoeffs = (cv::Mat_<float>(1, 5) << 0, 0, 0, 0, 0); // ToDelete?

/* Other parameters not necessary for now

in_video.open(source); // id
in_video.set(cv::CAP_PROP_FPS, 30);
in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
in_video.set(cv::CAP_PROP_SATURATION, 0);
*/

/************  ************/


/************ ROS2 Node ************/

auto debug_vect = px4_msgs::msg::IrlockReport();
auto irlock_msg = px4_msgs::msg::IrlockReport();
auto vis_odom = px4_msgs::msg::VehicleVisualOdometry();
auto get_odometry = px4_msgs::msg::VehicleOdometry();
int navstate = -1;
int LANDING_MARKER_ACTIVE = 6;
float MKR_SWITCH_AGL_ALT = 3.1;
float altitude_agl;
int distance_quality;
float x_origin = 0.0;
float y_origin = 0.0;
class DebugVectAdvertiser : public rclcpp::Node
{
public:
	DebugVectAdvertiser() : Node("offboard_odometry") {

#ifdef ROS_DEFAULT_API
		/*publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10);*/
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		vehicle_visual_odometry_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in", 10);
#else
		/*publisher_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in");*/
		offboard_control_mode_publisher_ =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in");
		trajectory_setpoint_publisher_ =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in");
		vehicle_command_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in");
		vehicle_visual_odometry_publisher_ =
			this->create_publisher<px4_msgs::msg::VehicleVisualOdometry>("fmu/vehicle_visual_odometry/in");
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
					//std::cout << "ts: "          << navstatmsg->timestamp    << std::endl;
					//std::cout << "CONTROL MODE: " << navstatmsg->nav_state << std::endl;
					/* RCLCPP_INFO(this->get_logger(), "\033[97m Control mode: %u \033[0m",
                                navstatmsg->nav_state);*/
					navstate = navstatmsg->nav_state;
				});
		vehicleodometry_sub_ =
			this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out", 10,
				[this](const px4_msgs::msg::VehicleOdometry::UniquePtr odometrymsg) {
					//std::cout << "ts: "          << navstatmsg->timestamp    << std::endl;
					//std::cout << "CONTROL MODE: " << navstatmsg->nav_state << std::endl;
					/* RCLCPP_INFO(this->get_logger(), "\033[97m Control mode: %u \033[0m",
                                navstatmsg->nav_state);*/
					get_odometry.x=odometrymsg->x;
					get_odometry.y=odometrymsg->y;
				});
		distancesensor_sub_ =
			this->create_subscription<px4_msgs::msg::DistanceSensor>("fmu/distance_sensor/out", 10,
				[this](const px4_msgs::msg::DistanceSensor::UniquePtr sensordistmsg) {
					//std::cout << "ts: "          << navstatmsg->timestamp    << std::endl;
					//std::cout << "CONTROL MODE: " << navstatmsg->nav_state << std::endl;
					/*RCLCPP_INFO(this->get_logger(), "\033[97m Altitude AGL: %f \033[0m",
                                sensordistmsg->current_distance);*/
					/*RCLCPP_INFO(this->get_logger(), "\033[97m OF Quality: %u \033[0m",
                                sensordistmsg->signal_quality);*/
					altitude_agl = sensordistmsg->current_distance;
					distance_quality = sensordistmsg->signal_quality;
				});
        offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]()->void {

            /* ROS2 */

            if (offboard_setpoint_counter_ == 100) {
				// Change to Offboard mode after 100 setpoints
				//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				// Arm the vehicle
				// this->arm();
			};

            // offboard_control_mode needs to be paired with trajectory_setpoint
			// MODES: 14 - OFFBOARD, 2 - POSITION, 4 - HOLD
			//publish_offboard_control_mode();
			//publish_velocity_setpoint();
			//publish_trajectory_setpoint();
			

           	// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 110) {
				offboard_setpoint_counter_++;
			};
			
			debug_vect.timestamp = timestamp_.load();
			
            //debug_vect.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
			//std::string name = "test";
			//std::copy(name.begin(), name.end(), debug_vect.name.begin());

            /* OpenCV script*/

			cv::Mat image, image_copy;
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f>> corners;
        	std::vector<int> ids_valid;
        	std::vector<std::vector<cv::Point2f>> corners_valid;
			std::vector<cv::Vec3d> rvecs, tvecs;
			
			//auto tvec = tvecs;
			/*if (navstate == 20){*/
				in_video.grab();
				in_video.retrieve(image);
				image.copyTo(image_copy);
				cv::aruco::detectMarkers(image, dictionary, corners, ids);
				//cv::aruco::drawDetectedMarkers(image_copy, corners_valid, ids_valid);
				
				//imshow("cam", image_copy); 
				ids_valid = ids;
        		corners_valid = corners;
			/*};*/
			if (altitude_agl < MKR_SWITCH_AGL_ALT) {
				LANDING_MARKER_ACTIVE = 6;
			} else {
				LANDING_MARKER_ACTIVE = 3;
			};
            int res_horizontal = image_copy.size().width;
            int res_vertical = image_copy.size().height;

            /*  DEBUGGING PRINT of camera input total pixels */
            // printf("Width: %i  Height: %i\n",horizontal_res,vertical_res);
            
            if (ids.size() > 0 /*&& navstate == 20*/) // If at least one marker detected
                {
				//for (int i = 0; i < int(ids.size()); i++){
					//if (int(ids.at(i)) == LANDING_MARKER_ACTIVE){
						//ldgmkrbig_index = i;
						//printf("Big marker detected!\n");

						//ids_valid.resize(1); Esto estaba jodiendo los calculos de rotacion
						//corners_valid.resize(1);
						//ids_valid[0]=ids.at(i);
						//corners_valid[0]=corners.at(i);

						/*Estimate pose function*/
	            		/* The output parameters rvecs and tvecs are the rotation and translation vectors respectively, for each of the markers in markerCorners.*/
                		/* The markerCorners parameter is the vector of marker corners returned by the detectMarkers() function.*/
						cv::aruco::estimatePoseSingleMarkers(corners, 0.40894, intrinsic_matrix, distCoeffs, rvecs, tvecs);
						for(unsigned long i=0; i<ids.size(); ++i){
							std::cout << i << "r1:" << rvecs[i][0] << "  r2:" << rvecs[i][1] << "  r3:" << rvecs[i][2] << "  r4:" << rvecs[i][3] << "  r5:" << rvecs[i][4] << "  r6:" << rvecs[i][5] << std::endl;
						};
						//cv::aruco::drawDetectedMarkers(image_copy, corners_valid, ids_valid);
						/* Draw axis for each marker detected*/
						// auto rvec = rvecs[0];
						auto tvec = tvecs[0];

						Eigen::Vector3f pos_in_cf; // position in the camera frame
            			Eigen::Vector4f pos_in_mf; // position in the marker frame
            			Eigen::Vector4f pos_in_bf; // position in the base frameri
            			Eigen::Matrix3f Rm; // rotation matrix: camera -> marker
            			Eigen::Matrix4f Hb; // homogeneous matrix: marker -> base
            			std::vector<Eigen::Vector4f> pos_vec; 
    					std::map<int,Eigen::Matrix4f> transMap; // Transformation map
    					std::map<int,tf2::Quaternion> q_bm; // Transformation map
							/*for(unsigned long i=0; i<ids.size(); ++i){
							tf2::Quaternion q,q_new;
							
							cv::Mat rotmat;
							cv::Rodrigues(rvecs,rotmat);		

							Eigen::Matrix<double, 3, 3> eigMat;
    							cv::eigen2cv(rotmat, eigMat);
    							Eigen::Quaterniond q(eigMat);					
							std::cout << rotmat << std::endl ;
							q.setRPY(rvecs[i][0],rvecs[i][1],rvecs[i][2]);
							
							// marker position in the camera frame
							pos_in_cf << tvecs[i][0], tvecs[i][1], tvecs[i][2];


							
							Hb = transMap[ids[i]];
							Rm = rotMat(q.w(), q.x(), q.y(), q.z());
							// std::cout << Rm << std::endl ;
							pos_in_mf << -Rm*pos_in_cf, 1;
							pos_in_bf = Hb*pos_in_mf;*/
							//std::cout << " PosX: " << pos_in_mf[0] << " PosY: "<< pos_in_mf[1] << " PosZ: " << pos_in_mf[2] << std::endl ;
							/*
							RCLCPP_INFO(this->get_logger(), "\033[97m x_opt: %f x_rx: %f y_opt: %f y_rx: %f z_opt: %f \033[0m",
									tvec[0], get_odometry.x, tvec[1], get_odometry.y ,tvec[2]);
							*/
							/*RCLCPP_INFO(this->get_logger(), "\033[97m x_opt: %f x_rx: %f y_opt: %f y_rx: %f z_opt: %f \033[0m",
									pos_in_bf[0], get_odometry.x, pos_in_bf[1], get_odometry.y ,pos_in_bf[2]);
							*/
						//};
						vis_odom.timestamp = timestamp_.load();
						vis_odom.timestamp_sample = timestamp_.load();
						vis_odom.x = tvec[0]-1;
						vis_odom.y = tvec[1];
						vis_odom.z = - altitude_agl;
						vis_odom.vx = NAN;
						vis_odom.vy = NAN;
						vis_odom.vz = NAN;
						vis_odom.local_frame = vis_odom.LOCAL_FRAME_NED;
						vis_odom.q[0] = NAN;
						vis_odom.q_offset[0] = NAN;
						vis_odom.pose_covariance[0] = NAN;
						vis_odom.pose_covariance[15] = NAN;
						vis_odom.rollspeed = NAN;
						vis_odom.pitchspeed = NAN;
						vis_odom.yawspeed = NAN;
						vis_odom.velocity_covariance[0] = NAN;
						vis_odom.velocity_covariance[15] = NAN;	

						vehicle_visual_odometry_publisher_->publish(vis_odom);
						//publish_vis_odom(float(tvec[0]),float(tvec[1]),float(-altitude_agl)); crashea
						// cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
						// int novalidmarkerflag = 0; //temp
					//};
				//};
				
                /* Draw the markers in the output picture - not necessary now*/
                /*cv::aruco::drawDetectedMarkers(image_copy, corners, ids);*/

                /* Draw axis for each marker detected - not necessary now */
                /*
                for (int i = 0; i < rvecs.size(); ++i)
                    {   
                    auto rvec = rvecs[i];
                    auto tvec = tvecs[i];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                    }
                */
                int marker_index = 0;
                //auto selected_marker = corners[marker_index];
				auto selected_marker = corners_valid[marker_index];
                auto corner1 = selected_marker[0]; // Top Left, small ref. red square
                auto corner2 = selected_marker[1]; // Clockwise or top right
                auto corner3 = selected_marker[2]; // Clockwise or bottom right
                auto corner4 = selected_marker[3]; // Clockwise or bottom left

                double x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
                double y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

                double x_avg = x_sum / 4;
                double y_avg = y_sum / 4;

                double x_dev = (x_avg - res_horizontal * .5) * fov_horiz / res_horizontal;
                double y_dev = (y_avg - res_vertical * .5) * fov_vert / res_vertical;

                /*  DEBUGGING PRINT of Deviation    */
                // printf("Xdev=%f Ydev=%f\n", x_dev, y_dev);
                debug_vect.pos_x = x_dev;
                debug_vect.pos_y = y_dev;
                debug_vect.signature = 1;
                debug_vect.size_x = 0;
                debug_vect.size_y = 0;
                //debug_vect.size_x = 4.0;
                //debug_vect.size_y = 4.0;
                /*  DEBUGGING PRINT of 
                printf("X1=%i X2=%i X3=%i X4=%i\nY1=%i Y2=%i Y3=%i Y4=%i\n", int(corner1.x), int(corner2.x), int(corner3.x), int(corner4.x),
                                                    int(corner1.y), int(corner2.y), int(corner3.y), int(corner4.y));*/
				irlock_msg = debug_vect;
				/*this->publisher_->publish(irlock_msg);*/
				/* ROS 2 Node console output for debugging*/
				/*RCLCPP_INFO(this->get_logger(), "\033[97m TargetDev: x: %f y: %f ID: %i ALT: %f \033[0m",
                                irlock_msg.pos_x, irlock_msg.pos_y, LANDING_MARKER_ACTIVE, altitude_agl);*/
				

                } else {
						vis_odom.timestamp = timestamp_.load();
						vis_odom.timestamp_sample = timestamp_.load();
						vis_odom.x = NAN;
						vis_odom.y = NAN;
						vis_odom.z = NAN;
						
						vis_odom.vx = NAN;
						vis_odom.vy = NAN;
						vis_odom.vz = NAN;
						vis_odom.local_frame = vis_odom.LOCAL_FRAME_NED;
						vis_odom.q[0] = NAN;
						vis_odom.q_offset[0] = NAN;
						vis_odom.pose_covariance[0] = NAN;
						vis_odom.pose_covariance[15] = NAN;
						vis_odom.rollspeed = NAN;
						vis_odom.pitchspeed = NAN;
						vis_odom.yawspeed = NAN;
						vis_odom.velocity_covariance[0] = NAN;
						vis_odom.velocity_covariance[15] = NAN;						

						vehicle_visual_odometry_publisher_->publish(vis_odom);
				};
            /* Image output for debugging - does not work here properly*/    
            // imshow("Detected markers", image_copy); 
		};
	timer_ = this->create_wall_timer(1ms, timer_callback);
	}
    void arm() const;
	void disarm() const;
private:
	rclcpp::TimerBase::SharedPtr timer_;
	/*rclcpp::Publisher<px4_msgs::msg::IrlockReport>::SharedPtr publisher_;*/
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleVisualOdometry>::SharedPtr vehicle_visual_odometry_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehiclestatus_sub_;
	rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr distancesensor_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicleodometry_sub_;
        
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped
    uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

    void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint() const;
	void publish_vis_odom(float x, float y, float z) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	Eigen::Matrix3f rotMat(double qw, double qx, double qy, double qz);				 
};

/**
 * @brief Send a command to Arm the vehicle
 */
void DebugVectAdvertiser::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void DebugVectAdvertiser::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void DebugVectAdvertiser::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

Eigen::Matrix3f DebugVectAdvertiser::rotMat(double qw, double qx, double qy, double qz) {
	// Auxiliary variables to avoid repeated arithmetic
	double qw2 = qw*qw, qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;
	double qxqy = qx*qy, qwqz = qw*qz, qwqy = qw*qy, qxqz = qx*qz, qyqz = qy*qz, qwqx = qw*qx;
	Eigen::Matrix3f rotM;
	rotM << qw2 + qx2 - qy2 - qz2, 2*(qxqy - qwqz), 2*(qwqy + qxqz),
			2*(qwqz + qxqy), qw2 - qx2 + qy2 - qz2, 2*(qyqz - qwqx),
			2*(qxqz - qwqy), 2*(qwqx + qyqz), qw2 - qx2 - qy2 + qz2;
	return rotM.transpose();
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void DebugVectAdvertiser::publish_trajectory_setpoint() const {
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	//msg.position = {nan, nan, nan};
	//msg.position = {0.0, 0.0, -5.0};
	msg.velocity = {0.3, 0.3, 0.5};
	msg.yaw = -3.14; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish a velocity setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */

void DebugVectAdvertiser::publish_vis_odom(float x, float y, float z) const {
	px4_msgs::msg::VehicleVisualOdometry msg{};
	msg.timestamp = timestamp_.load();
	msg.x = x;
	msg.y = y;
	msg.z = z;
	printf("About to publish  odometry!");
	vehicle_visual_odometry_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */

void DebugVectAdvertiser::publish_vehicle_command(uint16_t command, float param1,
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
	std::cout << std::fixed;
    std::cout << std::setprecision(3);
    in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugVectAdvertiser>());

    in_video.release();
    rclcpp::shutdown();
    return 0;
}
