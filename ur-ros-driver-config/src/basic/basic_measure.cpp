// librealsense dependencies
#include <librealsense2/rs.hpp>

// opencv dependencies
#include <opencv2/opencv.hpp>

// Misc headers
#include <csignal>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief Class to aid depth processing
 */ 
class ProcessVision{

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    float depth_scale_;

    const std::string rbgimg_window_name_ = "RGB Image";
    const std::string depthimg_window_name_ = "Colorized Depth Image";

public:

    /**
     * @brief Constructor
     * Set's up the parameters of the stream and starts the vision stream
     */ 
    ProcessVision(){
        try{
            cfg_.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            cfg_.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

            auto profile = pipe_.start(cfg_);
            auto sensor = profile.get_device().first<rs2::depth_sensor>();
            this->depth_scale_ = sensor.get_depth_scale();

            // Set the device to High Accuracy preset of the D400 stereoscopic cameras
            if (sensor && sensor.is<rs2::depth_stereo_sensor>()){
                sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
            }

            cv::namedWindow(this->rbgimg_window_name_, cv::WINDOW_AUTOSIZE);
            cv::namedWindow(this->depthimg_window_name_, cv::WINDOW_AUTOSIZE);
        }
        catch(const rs2::error& e){
            std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
        }
        catch (const std::exception& e){
            std::cerr << e.what() << std::endl;
        }
    }

    ~ProcessVision(){
        cv::destroyAllWindows();
    }

    /**
     * @brief Main function that performs processing. Executes post-processing filter on the
     * data obtained from the camera in a separate thread.
     */ 
    void run(){
        // alive boolean will signal the worker threads to finish-up
        std::atomic_bool alive{ true };
        rs2::frame_queue postprocessed_frames;

        // Declare depth colorizer for pretty visualization of depth data
        rs2::colorizer colorize;

        // Define transformations from and to Disparity domain
        rs2::disparity_transform depth_2_disparity(true);
        rs2::disparity_transform disparity_2_depth(false);

        // Define spatial filter (edge-preserving)
        rs2::spatial_filter spat;
        // Enable hole-filling
        // Hole filling is an agressive heuristic and it gets the depth wrong many times
        // However, this demo is not built to handle holes
        // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
        spat.set_option(RS2_OPTION_HOLES_FILL, 2); // 5 = fill all the zero pixels

        // Define temporal filter
        rs2::temporal_filter temp;

        // Spatially align all streams to depth viewport
        rs2::align align_to(RS2_STREAM_COLOR);

        for(int i = 0; i < 30; i++){
            //Wait for all configured streams to produce a frame
            this->pipe_.wait_for_frames();
        }
        // Video-processing thread will fetch frames from the camera,
        // apply post-processing and send the result to the main thread for rendering
        // It recieves synchronized (but not spatially aligned) pairs
        // and outputs synchronized and aligned pairs
        std::thread video_processing_thread([&]() {
            while(alive){
                // Fetch frames from the pipeline and send them for processing
                rs2::frameset data;
                if (this->pipe_.poll_for_frames(&data)){
                    // First make the frames spatially aligned
                    data = data.apply_filter(align_to);

                    // To make sure far-away objects are filtered proportionally
                    // we try to switch to disparity domain
                    data = data.apply_filter(depth_2_disparity);

                    // Apply spatial filtering
                    data = data.apply_filter(spat);

                    // Apply temporal filtering
                    data = data.apply_filter(temp);

                    // If we are in disparity domain, switch back to depth
                    data = data.apply_filter(disparity_2_depth);

                    // Apply color map for visualization of depth
                    data = data.apply_filter(colorize);

                    // Send resulting frames for visualization in the main thread
                    postprocessed_frames.enqueue(data);
                }
            }
        });

        rs2::frameset current_frameset;

        // main loop
        while(cv::waitKey(1) < 0){
            postprocessed_frames.poll_for_frame(&current_frameset);
            if(current_frameset){
                auto color_frame = current_frameset.get_color_frame();
                auto colorized_depth_frame =
                            current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

                const int w = color_frame.as<rs2::video_frame>().get_width();
                const int h = color_frame.as<rs2::video_frame>().get_height();

                cv::Mat color_image(cv::Size(w, h), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                cv::Mat colorized_depth_image(cv::Size(w, h), CV_8UC3, (void*)colorized_depth_frame.get_data(), cv::Mat::AUTO_STEP);

                cv::imshow(this->rbgimg_window_name_, color_image);
                cv::imshow(this->depthimg_window_name_, colorized_depth_image);
            }
        }
        video_processing_thread.join();
        alive = false;
    }

};


void signalHandler( int signum ) {
    cv::destroyAllWindows();
    exit(signum);  
}

int main(){
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ProcessVision pv;
    pv.run();

    return 0;
}