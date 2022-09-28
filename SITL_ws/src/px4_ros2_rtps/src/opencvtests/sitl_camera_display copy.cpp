/* 

File: sitl_camera_display.cpp 

Author: Manuel Tolino Contreras

Decription: This file contains the code intended to generate a node which should
capture a gstreamer video input and detect ArUco markers. The detected
markers should be represented in real time in a windows superposed with
the video feed.

*/


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp> 
#include <opencv2/highgui.hpp> 
#include <iostream>
#include <cstdlib>
#include <string>

namespace {
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
}
using namespace cv;

int MARKER_BIG_SIZEmm=409;
int MARKER_SMALL_SIZEmm=178; 

static void calcDev(std::vector<std::vector<cv::Point2f>> corners,
                    int marker_index, float res_horiz, float fov_horiz,
                    float res_vert, float fov_vert, float *xdev, float *ydev);

//calcular desviacion
static void calcDev(std::vector<std::vector<cv::Point2f>> corners,
                    int marker_index, float res_horiz, float fov_horiz,
                    float res_vert, float fov_vert, float *xdev, float *ydev)
                    { 
                    auto selected_marker = corners[marker_index];
                    auto corner1 = selected_marker[0]; // Top Left, small ref. red square
                    auto corner2 = selected_marker[1]; // Clockwise or top right
                    auto corner3 = selected_marker[2]; // Clockwise or bottom right
                    auto corner4 = selected_marker[3]; // Clockwise or bottom left

                    float x_sum = corner1.x + corner2.x + corner3.x + corner4.x ;
                    float y_sum = corner1.y + corner2.y + corner3.y + corner4.y ;

                    float x_avg = x_sum / 4;
                    float y_avg = y_sum / 4; 

                    float x_dev = (x_avg - res_horiz * .5) * fov_horiz / res_horiz;
                    float y_dev = (y_avg - res_vert * .5) * fov_vert / res_vert;
                    *xdev = x_dev;
                    *ydev = y_dev;
}




int main(int argc, char **argv)
{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);
    
    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");
    int wait_time = 10;
    
    cv::String videoInput = "0";
    cv::VideoCapture in_video;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
        char* end = nullptr;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
            in_video.open(videoInput); // url
        } else {
            in_video.open(source); // id
            in_video.set(cv::CAP_PROP_FPS, 30);
            in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
            in_video.set(cv::CAP_PROP_SATURATION, 0);
        }
    } else {
        in_video.open("udpsrc port=5600 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1");
        //in_video.set(cv::CAP_PROP_FPS, 30);
        //in_video.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        //in_video.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
        //in_video.set(cv::CAP_PROP_SATURATION, 0);
        printf("OPENED\n");
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }

    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    const cv::Mat  intrinsic_matrix = (cv::Mat_<float>(3, 3)
                               << 600,  0.0,         400,
                                  0.0,       600,    400,
                                  0.0,       0.0,                  1.0);                         
    /* NO distorsion for the virtual camera*/
    const cv::Mat  distCoeffs = (cv::Mat_<float>(5, 1) << 0.0, 0.0, 0.0, 0.0, 0.0);
    float fov_horiz = 1.570796327 ;
    float fov_vert = 1.570796327 ;
    int MARKER_BIG = 4;
    int MARKER_SMALL = 6;
    float bigmkr_devX, bigmkr_devY, smallmkr_devX, smallmkr_devY = 0.0 ;
    /* The output parameters rvecs and tvecs are the rotation and translation vectors respectively, for each of the markers in markerCorners.*/
    /* The markerCorners parameter is the vector of marker corners returned by the detectMarkers() function.*/
    std::vector<cv::Vec3d> rvecs, tvecs;

    // Text overlay parameters
    Point text1_position(80, 80);//Declaring the text position//
    Point text2_position(80, 140);//Declaring the text position//
    int font_size = 1;//Declaring the font size//
    Scalar font_Color(0, 255, 0);//Declaring the color of the font//
    int font_weight = 2;//Declaring the font weight//


    while (in_video.grab()) {
        cv::Mat image, image_copy;
        in_video.retrieve(image);
        image.copyTo(image_copy);

        cv::namedWindow("Detected markers", WINDOW_NORMAL); 
        // create a trackbar 
        cv::createTrackbar("Big marker size (mm)", "Detected markers", &MARKER_BIG_SIZEmm, 500); 

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids_valid;
        std::vector<std::vector<cv::Point2f>> corners_valid;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        //landingmarker_big = ids[0];
        ids_valid = ids;
        corners_valid = corners;
        int res_horizontal = image_copy.size().width;
        int res_vertical = image_copy.size().height;
        int novalidmarkerflag = 0;

        // If at least one marker detected
        if (ids.size() > 0)
            {
            for (int i = 0; i < int(ids.size()); i++){
                if (int(ids.at(i)) == MARKER_BIG){
                    ids_valid.resize(1);
                    corners_valid.resize(1);
                    ids_valid[0]=ids.at(i);
                    corners_valid[0]=corners.at(i);                    
                    float MKR_SIZE = MARKER_BIG_SIZEmm / 1000.0;
                    cv::aruco::estimatePoseSingleMarkers(corners_valid, MKR_SIZE, intrinsic_matrix, distCoeffs, rvecs, tvecs);
                    cv::aruco::drawDetectedMarkers(image_copy, corners_valid, ids_valid);
                    auto rvec = rvecs[0];
                    auto tvec = tvecs[0];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                    calcDev(corners_valid, 0, res_horizontal, fov_horiz, res_vertical, fov_vert, &bigmkr_devX, &bigmkr_devY);
                    novalidmarkerflag = 0;
                    //std::cout << i << ": r1:" << rvecs[i][0] << "  r2:" << rvecs[i][1] << "  r3:" << rvecs[i][2] << "  r4:" << rvecs[i][3] << "  r5:" << rvecs[i][4] << "  r6:" << rvecs[i][5] << std::endl;
                    
                } else if (ids.at(i) == MARKER_SMALL) {
                    ids_valid.resize(1);
                    corners_valid.resize(1);
                    ids_valid[0]=ids.at(i);
                    corners_valid[0]=corners.at(i);                    
                    float MKR_SIZE = MARKER_SMALL_SIZEmm / 1000.0;
                    cv::aruco::estimatePoseSingleMarkers(corners_valid, MKR_SIZE, intrinsic_matrix, distCoeffs, rvecs, tvecs);
                    cv::aruco::drawDetectedMarkers(image_copy, corners_valid, ids_valid);
                    auto rvec = rvecs[0];
                    auto tvec = tvecs[0];
                    cv::aruco::drawAxis(image_copy, intrinsic_matrix, distCoeffs, rvec, tvec, 0.1);
                    calcDev(corners_valid, 0, res_horizontal, fov_horiz, res_vertical, fov_vert, &smallmkr_devX, &smallmkr_devY);
                    novalidmarkerflag = 0;
                    //std::cout << i << ": r1:" << rvecs[i][0] << "  r2:" << rvecs[i][1] << "  r3:" << rvecs[i][2] << "  r4:" << rvecs[i][3] << "  r5:" << rvecs[i][4] << "  r6:" << rvecs[i][5] << std::endl;
                } else {
                    novalidmarkerflag = 1;
                };
            };
        };
        if (novalidmarkerflag == 0) {
            std::string str1 = "Big marker Deviation X: ";
            std::string str2 = std::to_string(bigmkr_devX);
            std::string str3 = " Deviation Y ";
            std::string str4 = std::to_string(bigmkr_devY);
            std::string overlaytext_bigmkr = str1 + str2 + str3 + str4;
            putText(image_copy, overlaytext_bigmkr, text1_position,FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);
            std::string str5 = "Small marker Deviation X: ";
            std::string str6 = std::to_string(smallmkr_devX);
            std::string str7 = " Deviation Y ";
            std::string str8 = std::to_string(smallmkr_devY);
            std::string overlaytext_smallmkr = str5 + str6 + str7 + str8;
            putText(image_copy, overlaytext_smallmkr, text2_position,FONT_HERSHEY_COMPLEX, font_size,font_Color, font_weight);
        };
        imshow("Detected markers", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;
    };
    cv::destroyWindow("Detected markers"); 
    in_video.release();
    return 0;
}