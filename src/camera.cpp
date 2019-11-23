//
// Created by chenghe on 11/22/19.
//
#include <camera.h>

namespace MegaDepth
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    Camera::
    Camera(const std::string& file_name)
    {
        Load(file_name);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    bool Camera::
    Load(const std::string& file_name)
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
        image_size_ = cv::Size(width, height);

        cv::Mat intrinsic = ( cv::Mat_<float> ( 3,3 ) << fx, 0, cx, 0, fy, cy, 0, 0, 1 );
        cv::Mat distortion = ( cv::Mat_<float> ( 1,5 ) << d0, d1, d2, d3, d4);

        std::cout<<"-------->Successfully load camera intrinsic!"<<std::endl;
        fs.release();

        intrinsic_ = cv::getOptimalNewCameraMatrix(intrinsic, distortion, image_size_, 0, image_size_, 0);
        cv::initUndistortRectifyMap(intrinsic, distortion, cv::Mat(), intrinsic_, image_size_, CV_16SC2, map1_, map2_);

        return true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat Camera::
    Undistort(const cv::Mat& distort_image) const
    {
        cv::Mat undistort_image;
        cv::remap(distort_image, undistort_image, map1_, map2_, cv::INTER_LINEAR);

        return undistort_image;
    }

}//end of MegaDepth