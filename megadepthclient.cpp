//
// Created by chenghe on 11/22/19.
//
#include <cv.hpp>
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

bool LoadYaml(const std::string& file_name, cv::Mat& intrinsic, cv::Mat& distortion, cv::Size& size)
{
    float fx, fy, cx, cy;
    float d0, d1, d2, d3, d4;
    int height, width;

    cv::FileStorage fs(file_name.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::string("Could not open file ") + file_name;
    cv::FileNode fn = fs["intrinsic"];
    fx = fn["fx"]; fy = fn["fy"]; cx = fn["cx"]; cy = fn["cy"];
    d0 = fn["d0"]; d1 = fn["d1"]; d2 = fn["d2"]; d3 = fn["d3"]; d4 = fn["d4"];
    height = fn["height"]; width = fn["width"];
    size = cv::Size(width, height);

    intrinsic = ( cv::Mat_<float> ( 3,3 ) << fx, 0, cx, 0, fy, cy, 0, 0, 1 );
    distortion = ( cv::Mat_<float> ( 1,5 ) << d0, d1, d2, d3, d4);

    std::cout<<"successfully load camera intrinsic!"<<std::endl;
    std::cout<<" fx = "<<fx<<" fy = "<<fy<<" cx = "<<cx<<" cy = "<<cy<<std::endl;
    std::cout<<" d0 = "<<d0<<" d1 = "<<d1<<" d2 = "<<d2<<" d3 = "<<d3<<" d4 = "<<d4<<std::endl;
    std::cout<<" height = "<<height<<" width = "<<width<<std::endl;
    fs.release();
    return true;
}

bool GetUndistortionMaps(const cv::Mat& intrinsic, const cv::Mat& distortion, const cv::Size& image_size,
        cv::Mat& new_intrinsic, cv::Mat& map1, cv::Mat& map2)
{
    new_intrinsic = cv::getOptimalNewCameraMatrix(intrinsic, distortion, image_size, 0, image_size, 0);
    cv::initUndistortRectifyMap(intrinsic, distortion, cv::Mat(), new_intrinsic, image_size, CV_16SC2, map1, map2);
}

PointCloud::Ptr ConstructPointCloud(const cv::Mat& color_image, const cv::Mat& inverse_depth_map, const cv::Mat& intrinsic)
{

    float fx, fy, cx, cy;
    fx = intrinsic.at<float>(0, 0);
    fy = intrinsic.at<float>(1, 1);
    cx = intrinsic.at<float>(0, 2);
    cy = intrinsic.at<float>(1, 2);

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

            cv::Vec3b color = color_image.at<cv::Vec3b>(row, col);
            point.b = color(0);
            point.g = color(1);
            point.r = color(2);

            point_cloud->points.push_back(point);
        }
    }

    return point_cloud;
}

int main(int argc,char* argv[])
{
    ///! check input parameters
    if (argc != 4)
    {
        std::cout<<"wrong parameters"<<std::endl;
        return 1;
    }

    ///!load camera parameters and undistion maps
    cv::Mat intrinsic, distortion, new_intrinsic, map1, map2;
    cv::Size image_size;
    LoadYaml(argv[3], intrinsic, distortion, image_size);
    GetUndistortionMaps(intrinsic, distortion, image_size, new_intrinsic, map1, map2);


    ///! some core data structures
    Timer timer;
    cv::VideoCapture video_capture;
    MegaDepth::MegaDepthEstimator mega_depth_estimator(argv[1]);
    video_capture.open(argv[2]);
    cv::Mat frame;
    bool stop(false);

    if(!video_capture.isOpened())

    {
        std::cout<<"video not open."<<std::endl;
        return 1;
    }

    ///! data structure for visualization
    cv::namedWindow("depth",CV_WINDOW_NORMAL);
    pcl::visualization::CloudViewer viewer("cloud");

    while(!stop)
    {
        if(!video_capture.read(frame))
        {
            break;
        }
        cv::remap(frame, frame, map1, map2, cv::INTER_LINEAR);
        ///! estimate depth
        timer.Tic();
        cv::Mat inverse_depth_map = mega_depth_estimator.Compute(frame);
        timer.Toc();

        ///! get point cloud
        PointCloud::Ptr point_cloud = ConstructPointCloud(frame, inverse_depth_map, new_intrinsic);
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