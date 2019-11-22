//
// Created by chenghe on 11/22/19.
//
#include <visualizer.h>

namespace MegaDepth
{
    //////////////////////////////////////////////////////////////////////////////////////////////
    Visualizer::
    Visualizer(const int image_height, const int image_width):
        s_cam_(pangolin::OpenGlRenderState(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                ///! camera position, model position, upper vector
                pangolin::ModelViewLookAt( 0, 0,-2, 0, 0, 0, pangolin::AxisY)
        )),
        handler_(pangolin::Handler3D(s_cam_))
    {
        pangolin::CreateWindowAndBind("MegaDepth Cloud Viewer",1024,768);
        glEnable(GL_DEPTH_TEST);
        float scale = static_cast<float>(image_width) / static_cast<float>(image_height);
        d_cam_ = pangolin::CreateDisplay()
                .SetBounds(1/3, 1, 0.0, 1.0)
                .SetHandler(&handler_);
        rgb_image_ = pangolin::Display("color_image")
                .SetBounds(0.0, 0.2, 0.0, 0.4, scale)
                .SetLock(pangolin::LockLeft, pangolin::LockBottom);

        depth_image_ = pangolin::Display("depth_image")
                .SetBounds(0.0, 0.2, 0.4, 0.8, scale)
                .SetLock(pangolin::LockLeft, pangolin::LockBottom);

        image_texture_ = pangolin::GlTexture(image_width, image_height,GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    void Visualizer::
    Draw(const cv::Mat& color_image,  const cv::Mat& inverse_depth_map,
         const cv::Mat& intrinsic, const std::string& text)
    {
        cv::Mat color_depth_map;
        color_depth_map = inverse_depth_map * 255;
        color_depth_map.convertTo(color_depth_map, CV_8U);
        cv::applyColorMap(color_depth_map, color_depth_map, cv::COLORMAP_JET);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        float fx, fy, cx, cy;
        fx = intrinsic.at<float>(0, 0);
        fy = intrinsic.at<float>(1, 1);
        cx = intrinsic.at<float>(0, 2);
        cy = intrinsic.at<float>(1, 2);


        ///! draw point cloud
        d_cam_.Activate(s_cam_);
        glPointSize(1);
        glBegin(GL_POINTS);
        for(size_t row = 0; row < static_cast<size_t >(inverse_depth_map.rows); ++row)
        {
            for(size_t col = 0; col < static_cast<size_t>(inverse_depth_map.cols); ++col)
            {
                float x, y, z;
                float inv_d = inverse_depth_map.at<float>(row, col);
                float depth = (1.0f / inv_d);

                x =  ((static_cast<float>(col) - cx) / fx) * depth;
                y = -((static_cast<float>(row) - cy) / fy) * depth;
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
        cv::putText(color_image, text, cv::Point2f(50, 50), cv::FONT_HERSHEY_DUPLEX , 2, cv::Scalar(0,0,255), 2, 16);
        image_texture_.Upload(color_image.data,GL_BGR,GL_UNSIGNED_BYTE);
        rgb_image_.Activate();
        glColor3f(1.0,1.0,1.0);
        image_texture_.RenderToViewportFlipY();

        image_texture_.Upload(color_depth_map.data,GL_BGR,GL_UNSIGNED_BYTE);
        depth_image_.Activate();
        glColor3f(1.0,1.0,1.0);
        image_texture_.RenderToViewportFlipY();

        pangolin::FinishFrame();
    }

}//end of MegaDepth

