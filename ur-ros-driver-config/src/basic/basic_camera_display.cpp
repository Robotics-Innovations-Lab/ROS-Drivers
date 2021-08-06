// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

// Misc headers
#include <csignal>

using namespace std;
using namespace cv;

void signalHandler( int signum ) {
    cv::destroyAllWindows();
    exit(signum);  
}

int main(){
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++){
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE );

    while(1){
        //Get each frame
        frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        
        imshow("Display Image", color);
        waitKey(1);
    }

    return 0;
}