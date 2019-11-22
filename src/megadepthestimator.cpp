//
// Created by chenghe on 11/22/19.
//
#include <megadepthestimator.h>


namespace MegaDepth
{

    ////////////////////////////////////////////////////////////////////////////////////////////
    MegaDepthEstimator::
    MegaDepthEstimator(const std::string &weight_path, const int height, const int width):
            height_(height),
            width_(width)
    {
        module_ = torch::jit::load(weight_path);
        module_->to(at::kCUDA);
        std::cout<<"success load MegaDepth network!"<<std::endl;
    }

    /////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat MegaDepthEstimator::
    Compute(const cv::Mat &image) const
    {
        const int original_height = image.rows;
        const int original_width  = image.cols;

        torch::Tensor input_tensor  = PreProcess(image);
        torch::Tensor output_tensor = module_->forward({input_tensor}).toTensor();
        torch::Tensor depth_tensor  = PostProcess(output_tensor);
        cv::Mat inverse_depth;
        cv::Mat(cv::Size(depth_tensor.size(1),depth_tensor.size(0)), CV_32FC1, depth_tensor.data<float>()).copyTo(inverse_depth);
        cv::resize(inverse_depth, inverse_depth, cv::Size(original_width, original_height));


        return inverse_depth;
    }

    /////////////////////////////////////////////////////////////////////////////////////////
    torch::Tensor MegaDepthEstimator::
    PreProcess(const cv::Mat &input_image) const
    {
        cv::Mat temp_image, float_image;
        cv::resize(input_image, temp_image, cv::Size(width_, height_));
        cv::cvtColor(temp_image,  temp_image, cv::COLOR_BGR2RGB);
        temp_image.convertTo(float_image, CV_32F, 1.0 / 255);
        std::vector<int64_t> dims = {1, height_, width_, 3};
        torch::Tensor input_tensor = torch::from_blob(float_image.data, dims, torch::kFloat32);
        input_tensor = input_tensor.permute({0,3,1,2});
        input_tensor = input_tensor.to(torch::kCUDA);

        return input_tensor;
    }

    /////////////////////////////////////////////////////////////////////////////////////////
    torch::Tensor
    MegaDepthEstimator::PostProcess(const torch::Tensor &output_tensor) const
    {
        torch::Tensor temp_tensor = output_tensor.squeeze();
        temp_tensor.exp_();
        torch::Tensor depth_tensor = 1.0 / temp_tensor;
        depth_tensor = depth_tensor / depth_tensor.max();
        depth_tensor = depth_tensor.to(torch::kCPU);
        depth_tensor = depth_tensor.to(torch::kFloat);

        return depth_tensor;
    }


}//end of MegaDepth
