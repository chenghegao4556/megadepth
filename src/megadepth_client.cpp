//
// Created by chenghe on 11/23/19.
//
#include <megadepth_client.h>

namespace MegaDepth
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    MegaDepthClient::
    MegaDepthClient(const std::string& weight_path, const std::string& video_path,
                    const std::string& camera_path):
                    camera_     (new MegaDepth::Camera(camera_path)),
                    estimator_  (new MegaDepth::MegaDepthEstimator(weight_path))
    {
        ///! initialize video capture
        video_capture_.open(video_path);
        if(!video_capture_.isOpened())

        {
            std::cout<<"--------> Video is not open."<<std::endl;
            exit(1);
        }

        ///! initialize pangolin
        image_height_ = camera_->GetSize().height;
        image_width_ = camera_->GetSize().width;

        pangolin::CreateWindowAndBind("MegaDepth Cloud Viewer",1024,768);
        glEnable(GL_DEPTH_TEST);
        pangolin::GetBoundWindow()->RemoveCurrent();

        intrinsic_ = camera_->GetIntrinsic();
        runing_ = true;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void MegaDepthClient::
    RenderLoop()
    {

        ///! initialize pangolin
        pangolin::BindToContext("MegaDepth Cloud Viewer");
        glEnable(GL_DEPTH_TEST);

        float scale = static_cast<float>(image_width_) / static_cast<float>(image_height_);
        pangolin::OpenGlRenderState s_cam = (pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                ///! camera position, model position, upper vector
                pangolin::ModelViewLookAt( 0, 0,-2, 0, 0, 0, pangolin::AxisNegY)));
        pangolin::Handler3D handler(s_cam);
        pangolin::View& d_cam = pangolin::CreateDisplay()
                .SetBounds(1/3, 1, 0.0, 1.0)
                .SetHandler(&handler);
        pangolin::View& rgb_image = pangolin::Display("color_image")
                .SetBounds(0.0, 0.2, 0.0, 0.4, scale)
                .SetLock(pangolin::LockLeft, pangolin::LockBottom);;
        pangolin::View& depth_image = pangolin::Display("depth_image")
                .SetBounds(0.0, 0.2, 0.4, 0.8, scale)
                .SetLock(pangolin::LockLeft, pangolin::LockBottom);;
        pangolin::GlTexture image_texture = pangolin::GlTexture(image_width_, image_height_,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);;

        pangolin::CreatePanel("menu").SetBounds(0.8,1,0.0,0.15);
        pangolin::Var<bool> pause("menu.Pause", false, false);
        pangolin::Var<std::string> fps("menu.FPS", " ");

        ///! initialize parameters for render
        float fx, fy, cx, cy;
        GetCameraParameters(fx, fy, cx, cy);

        cv::Mat inverse_depth_map;
        cv::Mat color_image;

        while(!ShouldQuit())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ///!get buff
            if(TryLock())
            {
                color_image = GetColorImage().clone();
                inverse_depth_map = GetInverseDepth().clone();
                fps = GetText();
                if(pangolin::Pushed(pause))
                {
                    if(!IsPause())
                    {
                        std::cout << "-------->System Pause" << std::endl;
                        SetPause();
                    }
                    else
                    {
                        std::cout << "-------->System UnPause" << std::endl;
                        ResetPause();
                    }
                }
                UnLock();
            }

            if(color_image.empty() ||
               inverse_depth_map.empty())
            {
                continue;
            }

            ///! colorize inverse depth map
            cv::Mat color_depth_map;
            color_depth_map = inverse_depth_map * 255;
            color_depth_map.convertTo(color_depth_map, CV_8U);
            cv::applyColorMap(color_depth_map, color_depth_map, cv::COLORMAP_JET);

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            ///! draw point cloud
            d_cam.Activate(s_cam);
            glPointSize(1);
            glBegin(GL_POINTS);
            for(size_t row = 0; row < static_cast<size_t >(inverse_depth_map.rows); ++row)
            {
                for(size_t col = 0; col < static_cast<size_t>(inverse_depth_map.cols); ++col)
                {
                    float x, y, z;
                    float inv_d = inverse_depth_map.at<float>(row, col);
                    float depth = (1.0f / inv_d);

                    x = ((static_cast<float>(col) - cx) / fx) * depth;
                    y = ((static_cast<float>(row) - cy) / fy) * depth;
                    z = depth;

                    cv::Vec3b color = color_image.at<cv::Vec3b>(row, col);
                    glColor3d(static_cast<float>(color(2))/255,
                              static_cast<float>(color(1))/255,
                              static_cast<float>(color(0))/255);
                    glVertex3d(x, y, z);
                }
            }
            glEnd();

            ///! draw color image and depth
            image_texture.Upload(color_image.data,GL_BGR,GL_UNSIGNED_BYTE);
            rgb_image.Activate();
            glColor3f(1.0,1.0,1.0);
            image_texture.RenderToViewportFlipY();

            image_texture.Upload(color_depth_map.data,GL_BGR,GL_UNSIGNED_BYTE);
            depth_image.Activate();
            glColor3f(1.0,1.0,1.0);
            image_texture.RenderToViewportFlipY();
            pangolin::FinishFrame();
        }
        pangolin::GetBoundWindow()->RemoveCurrent();
        runing_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void MegaDepthClient::
    MainLoop()
    {
        MegaDepth::Timer timer;
        while(IsRunning())
        {
            while(IsPause())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                if(!IsRunning())
                {
                    break;
                }
            }
            cv::Mat frame;
            if(video_capture_.read(frame))
            {
                frame = camera_->Undistort(frame);
                ///! estimate depth
                timer.Tic();
                cv::Mat inverse_depth_map = estimator_->Compute(frame);
                timer.Toc();

                ///!show fps of mega depth
                std::stringstream fps;
                fps<<setiosflags(std::ios::fixed)<<std::setprecision(2)<<(1.0 / timer.Duration());
                if(TryLock())
                {
                    SetText(fps.str());
                    SetColorImage(frame);
                    SetInverseDepth(inverse_depth_map);
                    UnLock();
                }
            }
        }
        video_capture_.release();
    }

}//end of MegaDepth