//
// Created by chenghe on 11/22/19.
//

#ifndef MEGADEPTH_CAMERA_H
#define MEGADEPTH_CAMERA_H

#include <memory>
#include <iostream>
#include <cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace MegaDepth
{
    class Camera
    {
    public:
        ///! smart pointer
        typedef std::shared_ptr<Camera> Ptr;
        typedef std::shared_ptr<const Camera> ConstPtr;

        /**
         * @brief constructor
         * @param[in] file_name
         */
        Camera(const std::string& file_name);

        /**
         * @brief load parameters from file
         * @param[in] file_name
         * @return success
         */
        bool Load(const std::string& file_name);

        /**
         * @brief undistort image
         * @param[in] distort_image
         * @return undistorted_image
         */
        cv::Mat Undistort(const cv::Mat& distort_image) const;

        /**
         * @brief get intrinsic matrix
         */
        const cv::Mat& GetIntrinsic() const
        {
            return intrinsic_;
        }

        /**
         * @brief get image size
         */
        const cv::Size& GetSize() const
        {
            return image_size_;
        }
    private:

        ///*******************private parameters******************///
        cv::Mat map1_;
        cv::Mat map2_;
        cv::Mat intrinsic_;
        cv::Size image_size_;
    };
}//end of MegaDepth

#endif //MEGADEPTH_CAMERA_H
