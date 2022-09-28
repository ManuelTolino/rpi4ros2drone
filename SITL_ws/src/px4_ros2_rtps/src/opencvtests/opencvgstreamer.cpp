#include <opencv2/opencv.hpp>

/// For the Raspberry Pi 64-bit Buster OS

int main()
{
    float f;
    float FPS[16];
    int   i,Fcnt=0;
    //some timing
    std::chrono::steady_clock::time_point Tbegin, Tend;
    //pipeline parameters
    int device = 0;                 //0=raspicam 1=usb webcam (when both are connected)
    int capture_width = 640 ;
    int capture_height = 480 ;
    int framerate = 30 ;
    int display_width = 640 ;
    int display_height = 480 ;

    //reset frame average
    for(i=0;i<16;i++) FPS[i]=0.0;

    cv::VideoCapture cap("udpsrc port=5666 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink drop=1", cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
        return (-1);
    }

    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;

    std::cout << "Hit ESC to exit" << "\n" ;
    while(true)
    {
    	if (!cap.read(frame)) {
            std::cout<<"Capture read error"<<std::endl;
            break;
        }
        //calculate frame rate
        Tend = std::chrono::steady_clock::now();
        f = std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
        Tbegin = Tend;
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        cv::putText(frame, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));

        //show frame
        cv::imshow("Camera",frame);

        char esc = cv::waitKey(5);
        if(esc == 27) break;
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}