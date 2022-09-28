#include <opencv2/opencv.hpp>

/// For the Raspberry Pi 64-bit Buster OS

std::string gstreamer_pipeline(int device, int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
            " v4l2src device=/dev/video"+ std::to_string(device) + " !"
            " video/x-raw,"
            " width=(int)" + std::to_string(capture_width) + ","
            " height=(int)" + std::to_string(capture_height) + ","
            " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
            " videoconvert ! videoscale !"
            " video/x-raw,"
            " width=(int)" + std::to_string(display_width) + ","
            " height=(int)" + std::to_string(display_height) + " ! appsink";
}

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
using namespace cv;
    //reset frame average
    for(i=0;i<16;i++) FPS[i]=0.0;

    std::string pipeline = gstreamer_pipeline(device,
                                              capture_width, capture_height, framerate,
                                              display_width, display_height);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n\n\n";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()) {
        std::cout<<"Failed to open camera."<<std::endl;
        return (-1);
    }
    //cv::VideoWriter videowriter("appsrc ! video/x-raw,format=BGR,width=640,height=480 ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=192.168.0.12 port=5666",cv::CAP_GSTREAMER,0,30,Size(640,480),true);

    //cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);

    cv::VideoWriter writer;
    /*writer.open("appsrc ! videoconvert ! x264enc noise-reduction=10000 tune=zerolatency byte-stream=true threads=4 ! mpegtsmux ! udpsink host=255.255.255.255 port=9999"
                , 0, (double)30, cv::Size(640, 480), true);
    */
   /* --- UDP SINK:
   writer.open("appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=512 byte-stream=true threads=1 ! mpegtsmux ! queue ! udpsink host=192.168.0.12 port=9999 sync=false"
                , 0, (double)30, cv::Size(1280, 720), true); 
                */
    writer.open("appsrc ! videoconvert ! video/x-raw, width=640, height=480, framerate=30/1 ! x264enc speed-preset=ultrafast tune=zerolatency bitrate=2048 byte-stream=true threads=1 ! matroskamux ! filesink location=output.mkv"
                , 0, (double)30, cv::Size(640, 480), true);
    if (!writer.isOpened()) {
        printf("=ERR= can't create video writer\n");
        return -1;
    }

    cv::Mat frame;
    cv::Mat image;
    int key;
    std::cout << "Hit ESC to exit" << "\n" ;
    while(true)
    {
    	if (!cap.read(frame)) {
            std::cout<<"Capture read error"<<std::endl;
            break;
        }
        if (frame.empty()) break;

        //calculate frame rate
        Tend = std::chrono::steady_clock::now();
        f = std::chrono::duration_cast<std::chrono::milliseconds> (Tend - Tbegin).count();
        Tbegin = Tend;
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        cv::putText(frame, cv::format("FPS %0.2f", f/16),cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));

        //show frame
        //cv::cvtColor(frame,image,cv::COLOR_RGB2BGR);
        cv::imshow("Camera Feed 1",frame);
        //videowriter.write(image);


        char esc = cv::waitKey(5);
        if(esc == 27) break;

        writer << frame;
        key = cv::waitKey( 30 );
    }

    cap.release();
    cv::destroyAllWindows() ;
    return 0;
}