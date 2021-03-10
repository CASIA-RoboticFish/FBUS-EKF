#ifndef FILTER_HPP
#define FILTER_HPP

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>

#include <Eigen/Core>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>

#include "common.hpp"
#include "matrix_math.hpp"
#include "imu_data_generator.hpp"
#include "GlogHelper.hpp"

using namespace Eigen;

#define IMU_BUFFER_MAX_SIZE 2000      // 缓存区的数据量上限
#define IMUDATA_NUM_FOR_INIT_POSE 200 // 用于IMU Pose初始化的数据数量

// 系统状态初始协方差
#define POSITION_CONV 0.0001
#define VELOCITY_CONV 0.01
#define QUATERNION_CONV 0.0001
#define ACCEL_BIAS_CONV 1e-2
#define GYRO_BIAS_CONV 1e-2
#define GRAVITY_CONV 100.0

namespace FBUSEKF
{
    // 单个Marker的检测结果
    struct DetectionResult
    {
        int markerID;
        double timeStamp;
        double markerSize;
        Eigen::Vector3d positionAtCL;      // Marker在左目相机（Left Camera）下的三维位置
        Eigen::Quaterniond quaternionM2CL; // Marker坐标系相对于左目相机系的四元数
        Eigen::Matrix3d rotmatM2L;         // Marker坐标系到左目相机系的旋转矩阵
        DetectionResult()
        {
            markerID = 0;
            timeStamp = 0;
            markerSize = 0;
            quaternionM2CL = Eigen::Quaterniond::Identity();
            positionAtCL = Eigen::Vector3d::Zero();
            rotmatM2L = Eigen::Matrix3d::Identity();
        }
    };
    typedef std::vector<DetectionResult> DetectionResultList; // 多个Marker检测结果，构成一个向量

    // 滤波器的主要类
    class FILTER
    {
    public:
        FILTER(FBUSEKF::CameraInfo leftCamera, FBUSEKF::CameraInfo rightCamera, FBUSEKF::IMUInfo imu, FBUSEKF::EkfParam ekfParam, FBUSEKF::MarkerPoseServer markerPoseServer)
        {
            this->cameraInfoList_.push_back(leftCamera);
            this->cameraInfoList_.push_back(rightCamera);
            Eigen::Matrix4d T_C_I; // Camera frame to Image frame
            T_C_I << -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            this->cameraInfoList_[0].T_SC = T_C_I * this->cameraInfoList_[0].T_SC;
            this->cameraInfoList_[1].T_SC = T_C_I * this->cameraInfoList_[1].T_SC;
            this->imuInfo_ = imu;
            this->ekfParam_ = ekfParam;
            // 是否已经对相机位姿完成了初始化
            this->isInitializePose_ = false;

            // Marker是否参与更新
            this->isMarkerCanBeUpdated_ = true;

            this->preUsedMarkerID_ = 0;
            
            // 初始化误差状态协方差矩阵
            this->sysErrorState_.stateCovariance = Eigen::MatrixXd::Zero(18, 18);
            for (int i = 0; i < 3; ++i)
                this->sysErrorState_.stateCovariance(i, i) = POSITION_CONV;
            for (int i = 3; i < 6; ++i)
                this->sysErrorState_.stateCovariance(i, i) = VELOCITY_CONV;
            for (int i = 6; i < 9; ++i)
                this->sysErrorState_.stateCovariance(i, i) = QUATERNION_CONV;
            for (int i = 9; i < 12; ++i)
                this->sysErrorState_.stateCovariance(i, i) = ACCEL_BIAS_CONV;
            for (int i = 12; i < 15; ++i)
                this->sysErrorState_.stateCovariance(i, i) = GYRO_BIAS_CONV;
            for (int i = 15; i < 18; ++i)
                this->sysErrorState_.stateCovariance(i, i) = GRAVITY_CONV;

            // 初始化IMU噪声协方差矩阵（这里可能存在一些问题，这里的数值设置可能不正确）
            this->sysErrorState_.noiseCovariance = Eigen::Matrix<double, 12, 12>::Zero();
            double dt = 0.001; // 1000Hz
            // this->sysErrorState_.noiseCovariance.block<3, 3>(0, 0) =
            //     Eigen::Matrix3d::Identity() * this->imuInfo_.sigma_a_c * this->imuInfo_.sigma_a_c;// * dt * dt;
            // this->sysErrorState_.noiseCovariance.block<3, 3>(3, 3) =
            //     Eigen::Matrix3d::Identity() * this->imuInfo_.sigma_g_c * this->imuInfo_.sigma_g_c;// * dt * dt;
            // this->sysErrorState_.noiseCovariance.block<3, 3>(6, 6) =
            //     Eigen::Matrix3d::Identity() * this->imuInfo_.sigma_ba * this->imuInfo_.sigma_ba;// * dt;
            // this->sysErrorState_.noiseCovariance.block<3, 3>(9, 9) =
            //     Eigen::Matrix3d::Identity() * this->imuInfo_.sigma_bg * this->imuInfo_.sigma_bg;// * dt;
            // this->sysErrorState_.noiseCovariance = this->sysErrorState_.noiseCovariance * 100;
            this->sysErrorState_.noiseCovariance.block<3, 3>(0, 0) =
                Eigen::Matrix3d::Identity() * ekfParam.accelNoiseCov;
            this->sysErrorState_.noiseCovariance.block<3, 3>(3, 3) =
                Eigen::Matrix3d::Identity() * ekfParam.gyroNoiseCov;
            this->sysErrorState_.noiseCovariance.block<3, 3>(6, 6) =
                Eigen::Matrix3d::Identity() * ekfParam.accelBiasCov;
            this->sysErrorState_.noiseCovariance.block<3, 3>(9, 9) =
                Eigen::Matrix3d::Identity() * ekfParam.gyroBiasCov;

            // 初始化观测噪声协方差矩阵
            this->sysErrorState_.observeNoiseCovariance = Eigen::Matrix<double, 7, 7>::Zero();
            this->sysErrorState_.observeNoiseCovariance.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * ekfParam.posNoiseCov;
            this->sysErrorState_.observeNoiseCovariance.block<4,4>(3,3) = Eigen::Matrix4d::Identity() * ekfParam.quatNoiseCov;

            // 初始化其他的辅助计算的矩阵
            this->sysErrorState_.stateJocobianMat = Eigen::Matrix<double, 18, 18>::Identity();
            this->sysErrorState_.noiseCoefficientMat = Eigen::Matrix<double, 18, 12>::Zero();
            this->sysErrorState_.noiseCoefficientMat.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

            //
            this->markerPoseServer_ = markerPoseServer;

            // 压入静态Marker的ID
            this->staticMarkerIDList_.push_back(0);

            // 压入动态Marker的ID
            this->dynamicMarkerIDList_.push_back(1);
            this->dynamicMarkerIDList_.push_back(2);

        }
        FILTER(const FBUSEKF::FILTER &) {}

        ~FILTER(){};

        // 回调函数，用于获取IMU数据 （这个是因为自己写了一个数据发生器，所以需要这个回调函数）
        static void InputIMUData(FBUSEKF::IMUData imudata, void *pObject);

	    // 获取相机位姿（可供其它线程调用）
        Eigen::Matrix4d GetCameraPose();
        Eigen::Matrix4d GetVisualPose();

        // 获取相机位姿（可供其它线程调用）
        void GetStaticMarkerPose(std::vector<Eigen::Matrix4d>& markerPoses);
        void GetDynamicMarkerPose(std::vector<Eigen::Matrix4d>& markerPoses);
        
        // 获取全部的用于可视化的信息（可供其它线程调用）
        void GetVisualizeInfo(Eigen::Matrix4d& cameraPose, Eigen::Matrix4d& visualPose, std::vector<Eigen::Matrix4d>& staticMarkerPoses, std::vector<Eigen::Matrix4d>& dynamicMarkerPoses);

        // IMU数据设置
        void SetImuData(FBUSEKF::IMUData);
        void SetImuDataUpdated()
        {
            this->isImuDataUpdated_ = true;
            // this->ekfCondVar.notify_one();
        }

        // 视觉检测结果 数据设置
        void SetDetectionResult(FBUSEKF::DetectionResultList &result);
        void SetDetectionResultUpdated()
        {
            this->isDetectionResultUpdated_ = true;
            this->ekfCondVar.notify_one();
        }

        // 线程相关函数
        void StartFilterThread();
        void JoinFilterThread()
        {
            this->filterThread_.join();
        }
        void FilterThreadFunction(); // 最主要的函数，这个函数会不断运行

    public:
        std::mutex imuMutex; // 这把锁主要用于锁住IMU Buffer，一旦有线程要操作IMU Buffer，都需要上这把锁
        std::mutex imgMutex; // 这把锁主要用于锁住视觉检测结果，一旦有线程要操作DetectionResultList，都需要上这把锁
        std::mutex visualMutex; // 这把锁主要用来锁住系统状态，主要是可视化线程需要获取相机位姿，需要用这把锁来保证操作的安全性
        std::mutex ekfMutex; // 这把锁主要是和条件变量配合，从而起到每当视觉检测结果到来时，就启动线程，开始执行
        std::condition_variable ekfCondVar;

    private:
        // 初始化重力矢量和陀螺仪的Bias
        // 方法： 假设刚启动程序时，系统保持静止。利用这一段时间的静止采集的数据，求取平均值，对重力矢量和陀螺仪Bias进行初始化。
        void InitializeGravityAndBias();

        // 初始化IMU系的Pose，也就是系统的位姿
        // 方法： 当视觉算法检测到Marker以后，将会启动本函数。利用检测到Marker前的一小段时间的IMU数据，同样假设IMU处于静止状态，
        // 计算加速度平均值，然后根据此时的加速度矢量和重力矢量对齐，从而计算出IMU的初始姿态，同时令IMU系的位置为0.
        bool InitializePose();
        
        // 重置系统状态（因为视觉偶尔会丢）
        void ResetSystemState();

        // 在状态向量中增加Marker的状态，并且扩增状态协方差矩阵
        // 方法：
        void StateAugmentation();
        
        // IMU时间更新
        // 方法：每次视觉算法检测结果到来以后，都会从上一次视觉检测时刻开始执行一次本函数。计算两次视觉检测结果之间的系统状态变化
        void BatchImuProcessing();
        void UpdateNominalState(double dtime, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);
        void UpdateCovariance(double dtime, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro);

        // 视觉量测更新
        // 方法：首先计算，理论上的量测量；其次，计算量测雅克比矩阵；再次，计算卡尔曼增益；再次，计算状态误差；再次，更新系统状态；最后，更新协方差矩阵。
        void ObservationUpdate();
        
        // 删除IMU缓存区中的数据
        void ClearImuBuffer(std::vector<FBUSEKF::IMUData>::iterator begin, std::vector<FBUSEKF::IMUData>::iterator end);

        // 滤波线程
        std::thread filterThread_;

        // 测量数据
        std::vector<FBUSEKF::IMUData> imuMeasuementBuffer_; // IMU数据缓冲区
        bool isImuDataUpdated_;

        // 视觉检测结果
        DetectionResultList detectionResult_; // 每次视觉检测的结果都会储存在这个列表中，一项代表一个Marker
        bool isDetectionResultUpdated_;
        int preUsedMarkerID_;

        // 系统状态
        FBUSEKF::NominalState sysNominalState_; // Nominal state
        FBUSEKF::ErrorState sysErrorState_;     // Error state
        bool isInitializePose_;
        bool isMarkerCanBeUpdated_;

        // 系统参数
        std::vector<FBUSEKF::CameraInfo> cameraInfoList_; // 相机参数
        FBUSEKF::IMUInfo imuInfo_; // IMU参数
        FBUSEKF::EkfParam ekfParam_;
        // Marker Server
        FBUSEKF::MarkerPoseServer markerPoseServer_;

        // 静态Marker（不会被更新的）
        std::vector<int> staticMarkerIDList_;

        // 动态Marker列表（会被更新的）
        std::vector<int> dynamicMarkerIDList_;

    };

}; // namespace FBUSEKF
#endif
