#ifndef VISION_HPP
#define VISION_HPP

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <aruco/aruco.h>
#include <aruco/dcf/dcfmarkertracker.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "common.hpp"
#include "matrix_math.hpp"
#include "GlogHelper.hpp"
#include "filter.hpp"

// 存储的最大的图像数
#define IMAGE_QUEUE_MAX_SIZE 10

namespace FBUSEKF
{
  // Marker数据类型
  struct Marker
  {
    int markerID;
    double timeStamp;
    double markerSize;
    bool isDetectedLeft;
    bool isDetectedRight;
    bool isTriangulation;
    bool isComputePose;
    Eigen::Vector3d positionAtCL;      // Marker在左目相机（Left Camera）下的三维位置
    Eigen::Quaterniond quaternionM2CL; // Marker坐标系相对于左目相机系的四元数
    Eigen::Matrix3d rotmatM2L;
    std::vector<cv::Point2f> cornerObservationLeft; // Marker的四个角点在图像中的坐标，已经经过畸变校正
    std::vector<cv::Point2f> cornerObservationRight;
    std::vector<Eigen::Vector3d> cornerPositionAtCL; // Marker的四个角点在左目相机系中的三维位置

    Marker()
    {
      markerID = 0;
      timeStamp = 0;
      markerSize = 0;
      isDetectedLeft = false;
      isDetectedRight = false;
      isTriangulation = false;
      isComputePose = false;
      quaternionM2CL = Eigen::Quaterniond::Identity();
      positionAtCL = Eigen::Vector3d::Zero();
      rotmatM2L = Eigen::Matrix3d::Identity();
      for (int i = 0; i < 4; i++)
      {
        cornerPositionAtCL.push_back(Eigen::Vector3d::Zero());
      }
    }
  };

  typedef int MarkerID;
  typedef std::map<MarkerID, Marker, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const MarkerID, Marker>>>
      MarkerObservation;

  class VISION
  {

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VISION(FBUSEKF::FILTER *pFilterObject, FBUSEKF::CameraInfo leftCamera, FBUSEKF::CameraInfo rightCamera, FBUSEKF::RefractInfo refractInfo, FBUSEKF::VisionParam visionParam)
    {
      //
      this->pFilter = pFilterObject;

      // 载入相机参数
      this->cameraInfo_.push_back(leftCamera);
      this->cameraInfo_.push_back(rightCamera);

      // 
      this->refractInfo_ = refractInfo;

      //
      this->visionParam_ = visionParam;
      // 是否启用Marker跟踪模式
      this->isTrackerApplied_ = this->visionParam_.trackingEnable;
      // 
      this->isUnderwaterMode_ = this->visionParam_.underwaterModeEnable;
      
      if (this->isTrackerApplied_)
      {
        // Tracker初始化
        this->markerTracker0_.setDictionary("ARUCO_MIP_36h12");
        this->markerTracker0_.loadParamsFromFile("config/arucoConfig.yml");
        this->markerTracker1_.setDictionary("ARUCO_MIP_36h12");
        this->markerDetector1_.loadParamsFromFile("config/arucoConfig.yml");
      }
      else
      {
        // Detector初始化#ifdef OPEN_UNDERWATER_MODE
        this->markerDetector0_.setDictionary("ARUCO_MIP_36h12");
        this->markerDetector0_.loadParamsFromFile("config/arucoConfig.yml");
        this->markerDetector1_.setDictionary("ARUCO_MIP_36h12");
        this->markerDetector1_.loadParamsFromFile("config/arucoConfig.yml");
      }

      // 检测参数
      this->markerSize_ = 0.28; // Marker的实际尺寸，按照需求修改，Unit: meter
      
      // 是否显示图像以及检测结果
      this->showImage_ = true;

      // 有效的Marker ID初始化（只有在这里effectiveMarkerList_中注册的Marker ID才是有效的Marker）
      this->effectiveMarkerList_.push_back(0);
      this->effectiveMarkerList_.push_back(1);
      this->effectiveMarkerList_.push_back(2);
      this->effectiveMarkerList_.push_back(3);
      this->effectiveMarkerList_.push_back(4);
      this->effectiveMarkerList_.push_back(5);
      this->effectiveMarkerList_.push_back(6);
      this->effectiveMarkerList_.push_back(7);
      this->effectiveMarkerList_.push_back(8);
      this->effectiveMarkerList_.push_back(16);
      this->effectiveMarkerList_.push_back(17);
      this->effectiveMarkerList_.push_back(18);


      if(this->effectiveMarkerList_.size() == 0)
      {
        LOG(ERROR) << "There is no any available marker that has been ergistered! ";
      }

      // 是否要重置Tracker
      this->isReDetect_ = false;
      
      // 初始化图像显示窗口
      cv::namedWindow("Aruco detector", CV_WINDOW_NORMAL);

      int isColor = 0;
      int fps = 25;
      int frameWidth = 1280;
      int frameHeight = 400;
      cv::VideoWriter aviWriter("test.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight), isColor);
      this->aviWriter_ = aviWriter;
    
    }

    VISION(const FBUSEKF::VISION &) {}

    cv::Mat GetImageData();

    void SetImageData(FBUSEKF::ImageData);

    void SetImageDataUpdated()
    {
      this->isImageDataUpdated_ = true;
      this->imageCondVar.notify_one();
    }

    void StartVisionThread();
    void JoinVisionThread()
    {
      this->visionThread_.join();
    }
    // 主线程函数
    void VisionThreadFunction();

    // 检测Aruco的Marker，并且获取双目图像中Marker四个角点的去除畸变后的归一化坐标
    void DetectArucoTag();
    
    // 利用双目图像对Marker四个角点的观测，三角化出角点在左目相机坐标系下的三维位置（空气中的三角化方法）
    void NormalTriangulation();

    // 利用双目图像对Marker四个角点的观测，三角化出角点在左目相机坐标系下的三维位置（水下的三角化方法，考虑折射模型）
    void RefractionTriangulation();

    // 利用四个角点的三维位置，计算出Marker的坐标系相对于左目坐标系的旋转，以及平移
    void ComputeMarkerPose();

    // According the marker velocity to reject some markers
    void RejectInvalidMarker();

    // 显示图像处理结果
    void DisplayImage();

    // 清除检测结果
    void ClearMarkerMap();

  public:
    std::mutex imageMutex;
    std::condition_variable imageCondVar;

  private:
    std::thread visionThread_;

    // 测量数据
    std::queue<FBUSEKF::ImageData> imageMeasuementQueue_; // 储存图像数据
    bool isImageDataUpdated_;                             // 是否接收到了新的图像

    bool isTrackerApplied_;
    aruco::MarkerDetector markerDetector0_;  // 左目相机的Marker检测子
    aruco::MarkerDetector markerDetector1_;  // 右目相机的Marker检测子
    aruco::DFCMarkerTracker markerTracker0_; // Tracker
    aruco::DFCMarkerTracker markerTracker1_;
    MarkerObservation markerObservation_; // Marker检测结果
    MarkerObservation preMarkerObservation_; 
    double markerSize_;

    // shi fou chong xin jian ce 
    bool isReDetect_;
    //
    std::vector<FBUSEKF::CameraInfo> cameraInfo_; // 相机参数
    
    //
    FBUSEKF::VisionParam visionParam_;

    // 
    FBUSEKF::RefractInfo refractInfo_;

    // 
    std::vector<FBUSEKF::MarkerID> effectiveMarkerList_;

    // 
    bool isUnderwaterMode_;

    //
    double preImageTime_;
    double fps_;     // 帧率
    bool showImage_; // 是否显示图像

    // 
    cv::VideoWriter aviWriter_;

    // 用于给Filter线程传递检测结果
    FBUSEKF::FILTER *pFilter;
  };

}; // namespace FBUSEKF

#endif
