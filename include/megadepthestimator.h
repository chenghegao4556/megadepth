//
// Created by chenghe on 11/22/19.
//

#ifndef MEGEDEPTH_MEGADEPTHESTIMATOR_H
#define MEGEDEPTH_MEGADEPTHESTIMATOR_H

#include <iostream>
#include <memory>
#include <torch/script.h>
#include <torch/torch.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace MegaDepth
{

    class MegaDepthEstimator
    {
    public:
        ///! smart pointer
        typedef std::shared_ptr<MegaDepthEstimator> Ptr;
        typedef std::shared_ptr<const MegaDepthEstimator> ConstPtr;

        /**
         * @brief constructor
         * @param weight_path
         * @param height
         * @param width
         */
        MegaDepthEstimator(const std::string& weight_path,
                           const int height = 384, const int width = 512);

        /**
         * @brief compute depth of input image
         * @param image
         * @return depth image
         */
        cv::Mat Compute(const cv::Mat& image) const;

    protected:

        /**
         * @brief convert image to tensor
         * @param input_image
         * @return tensor
         */
        torch::Tensor PreProcess(const cv::Mat& input_image) const;

        /**
         * @brief convert output of network to depth tensor
         * @param output_tensor
         * @return depth tensor
         */
        torch::Tensor PostProcess(const torch::Tensor& output_tensor) const;

    private:

        ///! input resolution
        int height_;
        int width_;

        ///! libtorch script
        std::shared_ptr<torch::jit::script::Module> module_;
    };//end of MegaDepthEstimator

}//end of MegaDepth


#endif //MEGEDEPTH_MEGADEPTHESTIMATOR_H
