//
// Created by chenghe on 11/22/19.
//
#include <chrono>
#include <sstream>
#include <pcl/visualization/cloud_viewer.h>
#include <megadepthestimator.h>

///! base datastructure for pcl visualizer
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

///! timer
class Timer
{
public:

    typedef std::chrono::steady_clock::time_point TimePoint;

    void Tic()
    {
        t1 = std::chrono::steady_clock::now();
    }

    void Toc()
    {
        t2 = std::chrono::steady_clock::now();
    }

    double Duration()
    {
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> ( t2-t1 );

        return time_used.count();
    }

private:
    TimePoint t1, t2;
};

int main(int argc,char* argv[])
{
    ///! check input parameters
    if (argc != 3)
    {
        std::cout<<"wrong parameters"<<std::endl;
        return 1;
    }

    const float cx = 690;
    const float cy = 230;
    const float fx = 980;
    const float fy = 980;

    Timer timer;
    cv::VideoCapture video_capture;
    MegaDepth::MegaDepthEstimator mega_depth_estimator(argv[1]);
    video_capture.open(argv[2]);

    if(!video_capture.isOpened())

    {
        std::cout<<"video not open."<<std::endl;
        return 1;
    }

    cv::namedWindow("depth",CV_WINDOW_NORMAL);
    pcl::visualization::CloudViewer viewer("cloud");
    cv::Mat frame;
    bool stop(false);

    while(!stop)
    {
        if(!video_capture.read(frame))
        {
            break;
        }
        ///! estimate depth
        timer.Tic();
        cv::Mat inverse_depth_map = mega_depth_estimator.Compute(frame);
        timer.Toc();

        ///! convert depth image to point cloud
        PointCloud::Ptr point_cloud( new PointCloud );
        for(size_t row = 0; row < static_cast<size_t >(inverse_depth_map.rows); ++row)
        {
            for(size_t col = 0; col < static_cast<size_t>(inverse_depth_map.cols); ++col)
            {
                PointT point;
                float inv_d = inverse_depth_map.at<float>(row, col);
                float depth = 1.0f / inv_d;

                point.x =  ((static_cast<float>(col) - cx) / fx) * depth;
                point.y = -((static_cast<float>(row) - cy) / fy) * depth;
                point.z = -depth;

                cv::Vec3b color = frame.at<cv::Vec3b>(row, col);
                point.b = color(0);
                point.g = color(1);
                point.r = color(2);

                point_cloud->points.push_back(point);
            }
        }

        ///! visualize point cloud and depth image

        ///! apply color map
        cv::Mat color_depth_map;
        color_depth_map = inverse_depth_map * 255;
        color_depth_map.convertTo(color_depth_map, CV_8U);
        cv::applyColorMap(color_depth_map, color_depth_map, cv::COLORMAP_JET);

        ///! merge original image and depth image
        cv::Mat output;
        output.push_back(frame);
        output.push_back(color_depth_map);

        ///!show fps of megadepth
        std::stringstream ss;
        ss<<"FPS "<<(1.0 / timer.Duration());
        cv::putText(output, ss.str(), cv::Point2f(50, 100), cv::FONT_HERSHEY_DUPLEX , 2, cv::Scalar(0,0,255), 2, 16);

        cv::imshow("depth", output);
        viewer.showCloud(point_cloud);

        if(static_cast<char>(cv::waitKey(1)) == 'q')
        {
            break;
        }
    }

    video_capture.release();
    cv::destroyAllWindows();

    return 0;
}