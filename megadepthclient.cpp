//
// Created by chenghe on 11/22/19.
//
#include <sstream>
#include <megadepthestimator.h>
#include <visualizer.h>
#include <camera.h>
#include <util.h>


int main(int argc,char* argv[])
{
    ///! check input parameters
    if (argc != 4)
    {
        std::cout<<"wrong parameters"<<std::endl;
        return 1;
    }


    ///! some core data structures
    MegaDepth::Timer timer;
    MegaDepth::Camera camera(argv[3]);
    MegaDepth::Visualizer visualizer(camera.GetSize().height, camera.GetSize().width);
    MegaDepth::MegaDepthEstimator mega_depth_estimator(argv[1]);

    cv::VideoCapture video_capture;
    video_capture.open(argv[2]);
    cv::Mat frame;

    if(!video_capture.isOpened())

    {
        std::cout<<"video not open."<<std::endl;
        return 1;
    }

    while(!pangolin::ShouldQuit())
    {
        if(!video_capture.read(frame))
        {
            break;
        }
        frame = camera.Undistort(frame);
        ///! estimate depth
        timer.Tic();
        cv::Mat inverse_depth_map = mega_depth_estimator.Compute(frame);
        timer.Toc();

        ///!show fps of megadepth
        std::stringstream ss;
        ss<<"FPS "<<(1.0 / timer.Duration());
        visualizer.Draw(frame, inverse_depth_map, camera.GetIntrinsic(), ss.str());
        if(static_cast<char>(cv::waitKey(1)) == 'q')
        {
            break;
        }
    }

    video_capture.release();
    cv::destroyAllWindows();

    return 0;
}