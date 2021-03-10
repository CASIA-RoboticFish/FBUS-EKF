#include "visualizer.hpp"

void FBUSEKF::VISUALIZER::VisualizerThreadFun()
{
    // fetch the context and bind it to this thread
    pangolin::BindToContext(this->winName);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(this->win_width_, this->win_height_, 420, 420, this->win_width_ / 2, this->win_height_ / 2, 0.2, 100), // 相机模型(宽,高,fx,fy,ux,uy,最后两个参数不知道)
        pangolin::ModelViewLookAt(-3, -1, 2, 0, -1, 2, pangolin::AxisY)                                                                     //
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, 0.0, 1.0, (double)(-(this->win_width_ / this->win_height_))) // 设置显示的范围,在图中的位置, SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                                .SetHandler(&handler);

#ifdef SHOW_IMAGE
    // Display image
    pangolin::View &cv_img = pangolin::Display("image")
                                 .SetBounds(2.0f / 3.0f, 1.0f, 0, 2.0f / 3.0f, (double)(this->win_width_ / this->win_height_))
                                 .SetLock(pangolin::LockLeft, pangolin::LockTop);
    pangolin::GlTexture imgTexture(this->image_width_, this->image_height_, GL_LUMINANCE8, false, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE); //Gray de wenti
#endif

    while (!pangolin::ShouldQuit())
    {
        const float w = 0.2;
        const float h = w * 0.75;
        const float z = w * 0.6;

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // 设置背景颜色
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        Eigen::Matrix4d cameraPose;
        Eigen::Matrix4d visualCameraPose;
        std::vector<Eigen::Matrix4d> staticMarkerPoseList;
        std::vector<Eigen::Matrix4d> dynamicMarkerPoseList;

        this->pFilter->GetVisualizeInfo(cameraPose, visualCameraPose, staticMarkerPoseList, dynamicMarkerPoseList);


        // 1. 绘制历史相机位姿
        if(this->isDrawHistoryPose)
        {
            for (auto iter = this->keyframePose_.begin(); iter != this->keyframePose_.end(); iter++)
            {
                glPushMatrix();
                glMultMatrixd(iter->data());
                glLineWidth(2);
                glBegin(GL_LINES);
                glColor3f(0.0f, 0.5f, 0.5f);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);
                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);
                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);
                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();
                glPopMatrix();
            }
        }

        // 2. 绘制历史运动轨迹
        if(this->isDrawTrajectory)
        {
            glLineWidth(2);
            glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f);
            int keyframeNum = this->keyframePose_.size();
            for (int i = 0; i < keyframeNum - 1; i++)
            {
                glVertex3f(this->keyframePose_[i][12], this->keyframePose_[i][13], this->keyframePose_[i][14]);
                glVertex3f(this->keyframePose_[i + 1][12], this->keyframePose_[i + 1][13], this->keyframePose_[i + 1][14]);
            }
            glEnd();
        }

        // 3. 绘制IMU-视觉融合的相机位姿
        if(this->isDrawCameraPose)
        {
            glPushMatrix();
            // Eigen::Matrix4d cameraPose = this->pFilter->GetCameraPose();
            std::vector<GLdouble> Twc = {cameraPose(0, 0), cameraPose(1, 0), cameraPose(2, 0), 0,
                                        cameraPose(0, 1), cameraPose(1, 1), cameraPose(2, 1), 0,
                                        cameraPose(0, 2), cameraPose(1, 2), cameraPose(2, 2), 0,
                                        cameraPose(0, 3), cameraPose(1, 3), cameraPose(2, 3), 1};
            glMultMatrixd(Twc.data());
            glLineWidth(4);
            glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 1.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);

            glColor3f(0.8f, 0.0f, 0.0f);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.1, 0.0, 0.0);

            glColor3f(0.0f, 0.8f, 0.0f);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 0.1, 0.0);

            glColor3f(0.2f, 0.2f, 1.0f);
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 0.0, 0.1);
            glEnd();
            glPopMatrix();

            std::vector<GLdouble> lastFramePose = this->keyframePose_.back();
            double dist = (lastFramePose[12] - Twc[12]) * (lastFramePose[12] - Twc[12]) +
                        (lastFramePose[13] - Twc[13]) * (lastFramePose[13] - Twc[13]) + (lastFramePose[14] - Twc[14]) * (lastFramePose[14] - Twc[14]);
            if (dist > 0.01) // 如果两帧之间相距超过10cm，那么记录这一帧的相机位姿
            {
                this->keyframePose_.push_back(Twc);
            }
            this->keyframeCnt_++;

            if (this->keyframePose_.size() > 100) // 如果储存的相机位姿超过100帧，那么丢弃前50帧
            {
                this->keyframePose_.erase(this->keyframePose_.begin(), this->keyframePose_.begin() + 50);
            }
        }

        // 4. 绘制仅仅基于视觉的相机位姿
        if(this->isDrawVsiualPose)
        {
            glPushMatrix();
            // Eigen::Matrix4d visualCameraPose = this->pFilter->GetVisualPose();
            std::vector<GLdouble> TwcVisual = {visualCameraPose(0, 0), visualCameraPose(1, 0), visualCameraPose(2, 0), 0,
                                            visualCameraPose(0, 1), visualCameraPose(1, 1), visualCameraPose(2, 1), 0,
                                            visualCameraPose(0, 2), visualCameraPose(1, 2), visualCameraPose(2, 2), 0,
                                            visualCameraPose(0, 3), visualCameraPose(1, 3), visualCameraPose(2, 3), 1};
            glMultMatrixd(TwcVisual.data());
            glLineWidth(4);
            glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3f(0, 0, 0);
            glVertex3f(w, h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, -h, z);
            glVertex3f(0, 0, 0);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(-w, h, z);
            glVertex3f(w, h, z);
            glVertex3f(-w, -h, z);
            glVertex3f(w, -h, z);

            glColor3f(0.8f, 0.0f, 0.0f); // Red X axis
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.1, 0.0, 0.0);

            glColor3f(0.0f, 0.8f, 0.0f); // Green Y axis
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 0.1, 0.0);

            glColor3f(0.2f, 0.2f, 1.0f); // Blue Z axi
            glVertex3f(0.0, 0.0, 0.0);
            glVertex3f(0.0, 0.0, 0.1);
            glEnd();
            glPopMatrix();
        }


        // 5. 绘制Static Marker
        if(this->isDrawStaticMarker)
        {
            // this->pFilter->GetStaticMarkerPose(staticMarkerPoseList);
            for (auto iter = staticMarkerPoseList.begin(); iter != staticMarkerPoseList.end(); iter++)
            {
                glPushMatrix();
                std::vector<GLdouble> Twm = {(*iter)(0, 0), (*iter)(1, 0), (*iter)(2, 0), 0,
                                            (*iter)(0, 1), (*iter)(1, 1), (*iter)(2, 1), 0,
                                            (*iter)(0, 2), (*iter)(1, 2), (*iter)(2, 2), 0,
                                            (*iter)(0, 3), (*iter)(1, 3), (*iter)(2, 3), 1};

                glMultMatrixd(Twm.data());

                glLineWidth(4);
                glBegin(GL_LINES);
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex3f(w, w, 0);
                glVertex3f(-w, w, 0);
                glVertex3f(w, w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(w, w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(-w, w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(-w, w, 0);

                glColor3f(0.8f, 0.0f, 0.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.5, 0.0, 0.0);

                glColor3f(0.0f, 0.8f, 0.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.0, 0.5, 0.0);

                glColor3f(0.2f, 0.2f, 1.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.0, 0.0, 0.5);
                glEnd();
                glPopMatrix();
            }
        }

        // 6. 绘制Dynamic Marker
        if(this->isDrawDynamicMarker)
        {
            // this->pFilter->GetStaticMarkerPose(dynamicMarkerPoseList);
            for (auto iter = dynamicMarkerPoseList.begin(); iter != dynamicMarkerPoseList.end(); iter++)
            {
                glPushMatrix();
                std::vector<GLdouble> Twm = {(*iter)(0, 0), (*iter)(1, 0), (*iter)(2, 0), 0,
                                            (*iter)(0, 1), (*iter)(1, 1), (*iter)(2, 1), 0,
                                            (*iter)(0, 2), (*iter)(1, 2), (*iter)(2, 2), 0,
                                            (*iter)(0, 3), (*iter)(1, 3), (*iter)(2, 3), 1};

                glMultMatrixd(Twm.data());

                glLineWidth(4);
                glBegin(GL_LINES);
                glColor3f(0.0f, 1.0f, 1.0f);
                glVertex3f(w, w, 0);
                glVertex3f(-w, w, 0);
                glVertex3f(w, w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(w, w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(w, -w, 0);
                glVertex3f(-w, w, 0);
                glVertex3f(-w, -w, 0);
                glVertex3f(-w, w, 0);

                glColor3f(0.8f, 0.0f, 0.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.5, 0.0, 0.0);

                glColor3f(0.0f, 0.8f, 0.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.0, 0.5, 0.0);

                glColor3f(0.2f, 0.2f, 1.0f);
                glVertex3f(0.0, 0.0, 0.0);
                glVertex3f(0.0, 0.0, 0.5);
                glEnd();
                glPopMatrix();
            }
        }
        
#ifdef SHOW_IMAGE
        cv::Mat img = this->pVision->GetImageData();
        if (img.flags != 0)
        {
            imgTexture.Upload(img.data, GL_LUMINANCE, GL_UNSIGNED_BYTE);
            cv_img.Activate();
            imgTexture.RenderToViewportFlipY();
        }
#endif

        // Delay
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}
