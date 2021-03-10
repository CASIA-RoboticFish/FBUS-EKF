#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP
#include <iostream>
#include <fstream> 
#include <thread>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "filter.hpp"
#include "vision.hpp"

//#define SHOW_IMAGE

namespace FBUSEKF
{


class VISUALIZER
{
public:
    VISUALIZER(FBUSEKF::FILTER* pFilterObject, FBUSEKF::VISION* pVisionObject)
    {
        this->pFilter = pFilterObject;
        this->pVision = pVisionObject;
        this->winName = "Underwater EKF";
        this->win_width_ = 1280;
        this->win_height_ = 960;
        this->image_width_ = 680;
        this->image_height_ = 400;
        this->keyframeCnt_ = 0;
        std::vector<GLdouble> Twc = {1,0,0,
                                     0,1,0,
                                     0,0,1,
                                     0,0,0};
        this->keyframePose_.push_back(Twc);
        pangolin::CreateWindowAndBind(this->winName, this->win_width_, this->win_height_);

        this->isDrawCameraPose = true;
        this->isDrawHistoryPose = false;
        this->isDrawTrajectory = true;
        this->isDrawStaticMarker = true;
        this->isDrawDynamicMarker = true;
        this->isDrawVsiualPose = true;

        glEnable(GL_DEPTH_TEST);
        pangolin::GetBoundWindow()->RemoveCurrent();
    }

    ~VISUALIZER(){};

    void StartVisualizerThread()
    {
        this->visualizerThread_ = std::thread(std::bind(&FBUSEKF::VISUALIZER::VisualizerThreadFun,this));
    }
    
    void JoinVisualizerThread()
    {
        this->visualizerThread_.join();
    }


private:
    std::string winName;
    std::thread visualizerThread_;
    void VisualizerThreadFun();
    
    std::vector<std::vector<GLdouble>> keyframePose_;
    size_t keyframeCnt_;

    // 控制显示哪些内容
    bool isDrawCameraPose;
    bool isDrawHistoryPose;
    bool isDrawTrajectory;
    bool isDrawStaticMarker;
    bool isDrawDynamicMarker;
    bool isDrawVsiualPose;

    //
    int win_width_;
    int win_height_;
    int image_width_;
    int image_height_;
    FBUSEKF::FILTER* pFilter;
    FBUSEKF::VISION* pVision;

};


};

#endif
