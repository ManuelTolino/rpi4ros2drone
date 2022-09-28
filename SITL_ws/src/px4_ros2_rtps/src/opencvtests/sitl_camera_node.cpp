#include <cstdlib>

#include <stdint.h>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/irlock_report.hpp>
#include <px4_msgs/msg/timesync.hpp>

using std::placeholders::_1;
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

//***********************************************//

cv::VideoCapture in_video;
cv::Mat image, image_copy;

auto debug_vect = px4_msgs::msg::IrlockReport();
auto irlock_msg = px4_msgs::msg::IrlockReport();

//***********************************************//



//***********************************************//

class OpenCVExp : public rclcpp::Node
{
public:
  OpenCVExp(): Node("Opencvfeed")
  {
    #ifdef ROS_DEFAULT_API
		irlock_report_pub_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in", 10);
    #else
		irlock_report_pub_ = this->create_publisher<px4_msgs::msg::IrlockReport>("fmu/irlock_report/in");
    #endif
        timesync_sub_ =
        this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
            [this](const px4_msgs::msg::Timesync::UniquePtr msg) {
                timestamp_.store(msg->timestamp);
            });
            //imgproc();
                /*
                in_video.grab();
                in_video.retrieve(image);
                image.copyTo(image_copy);
                imshow("Camera feed", image_copy);
                char key = (char)cv::waitKey(3);*/
				debug_vect.timestamp = timestamp_.load();
                debug_vect.pos_x = 0.0;
                debug_vect.pos_y = 0.0;
                debug_vect.signature = 1;
                debug_vect.size_x = 0;
                debug_vect.size_y = 0;
				irlock_msg = debug_vect;
				this->irlock_report_pub_->publish(irlock_msg);
  }
private:
    rclcpp::Publisher<px4_msgs::msg::IrlockReport>::SharedPtr irlock_report_pub_;
    rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

    void imgproc() {
    if (in_video.grab()){
        in_video.retrieve(image);
        image.copyTo(image_copy);
        imshow("Camera feed", image_copy);
        char key = (char)cv::waitKey(3);
        if (key == 27) 
            return;
    }
}
};


// =============================================================================
// main
int main(int argc, char * argv[])
{
  //in_video.open("udpsrc port=5601 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
  // Act on the parameters and other stuff
  rclcpp::init(argc, argv);

  // Create the ROS node and start it up
  rclcpp::spin(std::make_shared<OpenCVExp>());

  rclcpp::shutdown();
  return 0;
}