//
// Created by chenghe on 11/22/19.
//

#ifndef MEGADEPTH_VISUALIZER_H
#define MEGADEPTH_VISUALIZER_H

#include <memory>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
namespace MegaDepth
{
    class Visualizer
    {
    public:
        ///! smart pointer
        typedef std::shared_ptr<Visualizer> Ptr;
        typedef std::shared_ptr<const Visualizer> ConstPtr;

        /**
         * @brief constructor
         * @param[in] image_height
         * @param[in] image_width
         */
        Visualizer(const int image_height, const int image_width);
        /**
         * @brief delete default constructor
         */
        Visualizer() = delete;

        /**
         * @brief visualize image, depth and point cloud
         * @param[in] color_image
         * @param[in]]inverse_depth_map
         * @param[in] intrinsic
         * @param[in] text
         */
        void Draw(const cv::Mat& color_image,  const cv::Mat& inverse_depth_map,
                  const cv::Mat& intrinsic, const std::string& text);

    private:

        ///*******************private parameters******************///
        pangolin::OpenGlRenderState s_cam_;
        pangolin::Handler3D handler_;
        pangolin::View d_cam_;
        pangolin::View rgb_image_;
        pangolin::View depth_image_;
        pangolin::GlTexture image_texture_;
    };//end of Visualizer

}//end of MegaDepth

#endif //MEGADEPTH_VISUALIZER_H
