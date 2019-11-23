//
// Created by chenghe on 11/23/19.
//
#ifndef MEGADEPTH_MEGADEPTH_CLIENT_H
#define MEGADEPTH_MEGADEPTH_CLIENT_H
#include <pangolin/pangolin.h>
#include <iomanip>
#include <sstream>
#include <megadepth_estimator.h>
#include <camera.h>
#include <util.h>

namespace MegaDepth
{
    class MegaDepthClient
    {
    public:

        /**
         * @brief delete default constructor
         */
        MegaDepthClient() = delete;

        /**
         * @brief constructor
         * @param[in] weight_path
         * @param[in] video_path
         * @param[in] camera_path
         */
        MegaDepthClient(const std::string& weight_path, const std::string& video_path,
                        const std::string& camera_path);

        /**
         * @brief start two threads
         */
        void Start()
        {
            std::thread main_loop(std::bind(&MegaDepthClient::MainLoop, this));
            std::thread render_loop(std::bind(&MegaDepthClient::RenderLoop, this));
            render_loop.join();
            main_loop.join();
        }

    protected:

        /**
         * @brief return should quit?
         */
        bool ShouldQuit() const
        {
            return pangolin::ShouldQuit();
        }
        /**
         * @brief render point cloud and image
         */
        void RenderLoop();

        /**
         * @brief main loop for estimate depth
         */
        void MainLoop();

        /**
         * @brief is pause?
         * @return
         */
        bool IsPause() const
        {
            return pause_;
        }

        void SetPause()
        {
            pause_ = true;
        }

        void ResetPause()
        {
            pause_ = false;
        }

        /**
        * @brief is running?
        */
        bool IsRunning() const
        {
            return runing_;
        };

        /**
         * @brief set text
         */
        void SetText(const std::string& text)
        {
            text_ = text;
        }

        /**
         * @brief get text
         */
        std::string GetText() const
        {
            return text_;
        }

        /**
         * @brief set color image
         */
        void SetColorImage(const cv::Mat& color_image)
        {
            color_image_buff_ = color_image.clone();
        }

        /**
         * @brief get color image
         */
        const cv::Mat& GetColorImage() const
        {
            return color_image_buff_;
        }

        /**
         * @brief set inverse depth
         */
        void SetInverseDepth(const cv::Mat& inverse_depth)
        {
            inverse_depth_buff_ = inverse_depth.clone();
        }

        /**
         * @brief get inverse depth
         */
        const cv::Mat& GetInverseDepth() const
        {
            return inverse_depth_buff_;
        }

        /**
         * @brief lock
         */
        bool TryLock()
        {
            return mutex_.try_lock();
        }

        /**
         * @brief unlock
         */
        void UnLock()
        {
            mutex_.unlock();
        }

        /**
         * @brief get focal length and principle point
         */
        void GetCameraParameters(float& fx, float& fy, float& cx, float& cy) const
        {
            fx = intrinsic_.at<float>(0, 0);
            fy = intrinsic_.at<float>(1, 1);
            cx = intrinsic_.at<float>(0, 2);
            cy = intrinsic_.at<float>(1, 2);
        }
    private:

        ///!core
        MegaDepth::Camera::Ptr camera_;
        MegaDepth::MegaDepthEstimator::Ptr estimator_;
        cv::VideoCapture video_capture_;

        ///! data buff
        cv::Mat color_image_buff_;
        cv::Mat inverse_depth_buff_;
        std::string text_;
        cv::Mat intrinsic_;

        ///! control
        std::mutex mutex_;
        bool runing_;
        bool pause_;

        int image_height_;
        int image_width_;
    };//end of MegaDepthClient
}//end of MegaDepth

#endif //MEGADEPTH_MEGADEPTH_CLIENT_H
