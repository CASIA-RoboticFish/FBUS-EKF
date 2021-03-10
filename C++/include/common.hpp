#ifndef COMMON_HPP
#define COMMON_HPP
#include <Eigen/Core>
#include <Eigen/Dense>
#include "yaml_eigen_converter.hpp"

#define PANGOLIN_ENABLE 1
// 用于控制是否打开可视化线程
#define OPEN_VISUALIZER_THREAD
// 
#define OPEN_DATA_RECORDING
//#define OPEN_IMAGE_RECORDING

#define M_PI 3.1415926

namespace FBUSEKF // Underwater Vision IMU Depth EKF
{
    // Marker
    struct MarkerPose
    {
        int markerID;
        Eigen::Vector3d positionAtG;
        Eigen::Quaterniond quaternionM2G;
        MarkerPose():markerID(-1),positionAtG(Eigen::Vector3d::Zero()),quaternionM2G(1, 0, 0, 0){}
    };

    typedef int MarkerID;
    typedef std::map<MarkerID, MarkerPose, std::less<int>, Eigen::aligned_allocator<
        std::pair<const MarkerID, MarkerPose> > > MarkerPoseServer;

    //
    struct EkfParam
    {
        double accelNoiseCov;
        double gyroNoiseCov;
        double accelBiasCov;
        double gyroBiasCov;
        double posNoiseCov;
        double quatNoiseCov;
        double fcpMarkerMaxDist; // ftp:filter control parameter
        double fcpMarkerSwitchThres;
        EkfParam() {}
        EkfParam(const YAML::Node &node)
        {
            accelNoiseCov = node["System Noise Covariance"]["accel_n_cov"].as<double>();
            gyroNoiseCov = node["System Noise Covariance"]["gyro_n_cov"].as<double>();
            accelBiasCov = node["System Noise Covariance"]["accel_b_cov"].as<double>();
            gyroBiasCov = node["System Noise Covariance"]["gyro_b_cov"].as<double>();
            posNoiseCov = node["Measure Noise Covariance"]["pos_n_cov"].as<double>();
            quatNoiseCov = node["Measure Noise Covariance"]["quat_n_cov"].as<double>();
            fcpMarkerMaxDist = node["Filter Control Param"]["marker_max_dist"].as<double>();
            fcpMarkerSwitchThres = node["Filter Control Param"]["marker_switch_thres"].as<double>();
        }
    };


    // 相机参数
    struct CameraInfo
    {
        double width;
        double height;
        Eigen::Vector2d focal_length;
        Eigen::Vector2d principal_point;
        Eigen::Matrix4d T_SC;
        Eigen::Matrix3d R;
        Eigen::Matrix<double, 3, 4> P;
        Eigen::Matrix3d K; // camera matrix
        Eigen::Vector4d D; // distortion coeff
        CameraInfo() {}
        CameraInfo(const YAML::Node &node)
        {
            width = node["width"].as<double>();
            height = node["height"].as<double>();
            focal_length = node["focal_length"].as<Eigen::Vector2d>();
            principal_point = node["principal_point"].as<Eigen::Vector2d>();
            T_SC = node["TSC"].as<Eigen::Matrix4d>();
            R = node["R"].as<Eigen::Matrix3d>();
            P = node["P"].as<Eigen::Matrix<double, 3, 4>>();
            K = node["K"].as<Eigen::Matrix3d>();
            D = node["D"].as<Eigen::Vector4d>();
        }
    };
    
    //
    struct RefractInfo
    {
        double waterRefRatio;
        double glassRefRatio;
        double airRefRatio;
        double airThickness;
        double glassThickness;
        Eigen::Vector3d normalVec;
        RefractInfo() {}
        RefractInfo(const YAML::Node &node)
        {
            airRefRatio = node["air_refract_ratio"].as<double>();
            waterRefRatio = node["water_refract_ratio"].as<double>();
            glassRefRatio = node["glass_refract_ratio"].as<double>();
            airThickness = node["air_thickness"].as<double>();
            glassThickness = node["glass_thickness"].as<double>();
            normalVec = node["normal_vector"].as<Eigen::Vector3d>();
        }
    };

    //  
    struct VisionParam
    {
        int frameWidth;
        int frameHeight;
        bool trackingEnable;
        bool underwaterModeEnable;
        double edgeForbiddenThres;
        double markerSize;
        double markerSizeThres;
        double markerSizeDiffThres;
        double markerMoveVelThres;
        double cornersProjDistThres;
        double makrerDectDistThres;
        VisionParam(){}
        VisionParam(const YAML::Node &node)
        {
            frameWidth = node["framewidth"].as<int>();
            frameHeight = node["frameheight"].as<int>();
            trackingEnable = node["tracking_enable"].as<bool>();
            underwaterModeEnable = node["underwater_mode_enable"].as<bool>();
            edgeForbiddenThres = node["edge_forbidden_thres"].as<double>();
            markerSize = node["markersize"].as<double>();
            markerSizeThres = node["markersize_thres"].as<double>();
            markerSizeDiffThres = node["markersize_diff_thres"].as<double>();
            markerMoveVelThres = node["marker_movevel_thres"].as<double>();
            cornersProjDistThres = node["corners_proj_dist_thres"].as<double>();
            makrerDectDistThres = node["makrer_dect_dist_thres"].as<double>();
        }
    };

    // IMU参数
    struct IMUInfo
    {
        double a_max;
        double g_max;
        double sigma_g_c; // noise
        double sigma_a_c; // noise
        double sigma_bg;  // zero bias noise  
        double sigma_ba;  // zero bias noise
        double sigma_gw_c; // drifts noise
        double sigma_aw_c; // drifts noise
        double tau;
        double g;
        Eigen::Vector4d a0;
        Eigen::Matrix4d T_BS;
        Eigen::Matrix<double, 3, 4> Acc;
        Eigen::Matrix<double, 3, 4> Gyr;

        IMUInfo() {}
        IMUInfo(const YAML::Node &node)
        {
            a_max = node["a_max"].as<double>();
            g_max = node["g_max"].as<double>();
            sigma_g_c = node["sigma_g_c"].as<double>();
            sigma_a_c = node["sigma_a_c"].as<double>();
            sigma_bg = node["sigma_bg"].as<double>();
            sigma_ba = node["sigma_ba"].as<double>();
            sigma_gw_c = node["sigma_gw_c"].as<double>();
            sigma_aw_c = node["sigma_aw_c"].as<double>();
            tau = node["tau"].as<double>();
            g = node["g"].as<double>();

            a0 = node["a0"].as<Eigen::Vector4d>();
            T_BS = node["T_BS"].as<Eigen::Matrix4d>();
            Acc = node["Acc"].as<Eigen::Matrix<double, 3, 4>>();
            Gyr = node["Gyr"].as<Eigen::Matrix<double, 3, 4>>();
        }
    };

    // IMU数据
    struct IMUData
    {
        double timeStamp;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        IMUData() : timeStamp(0), accel(Eigen::Vector3d::Zero()), gyro(Eigen::Vector3d::Zero()) {}
        IMUData(double t, Eigen::Vector3d acceldata, Eigen::Vector3d gyrodata) : timeStamp(t), accel(acceldata), gyro(gyrodata) {}
        IMUData(double t, double wx, double wy, double wz, double ax, double ay, double az)
        {
            timeStamp = t;
            accel(0) = ax;
            accel(1) = ay;
            accel(2) = az;
            gyro(0) = wx;
            gyro(1) = wy;
            gyro(2) = wz;
        }
    };

    // Image数据
    struct ImageData
    {
        double timeStamp;
        cv::Mat image0; // 左图（Left）
        cv::Mat image1; // 右图（Right）
    };


    // 系统标称状态
    struct NominalState
    {
        double timeStamp;
        Eigen::Quaterniond quaternionI2G;
        Eigen::Matrix3d rotmatI2G;
        Eigen::Vector3d eulerAngle;
        Eigen::Vector3d positionAtG;
        Eigen::Vector3d velocityAtG;
        Eigen::Vector3d accelBias;
        Eigen::Vector3d gyroBias;
        Eigen::Vector3d gravityAtG;
	
	    // 基于视觉信息计算的系统状态
        Eigen::Vector3d positionOnlyVisual;
        Eigen::Quaterniond quaternionOnlyVisual;
        

        NominalState() : timeStamp(0), quaternionI2G(1, 0, 0, 0), rotmatI2G(Eigen::Matrix3d::Zero()), eulerAngle(Eigen::Vector3d::Zero()),
                         positionAtG(Eigen::Vector3d::Zero()), velocityAtG(Eigen::Vector3d::Zero()), accelBias(Eigen::Vector3d::Zero()),
                         gyroBias(Eigen::Vector3d::Zero()), gravityAtG(Eigen::Vector3d::Zero()) {}
    };

    // 系统误差状态
    struct ErrorState
    {
        double timeStamp;
        Eigen::Quaterniond deltaQuaternion;
        Eigen::AngleAxisd deltaTheta;
        Eigen::Vector3d deltaPosition;
        Eigen::Vector3d deltaVelcity;
        Eigen::Vector3d deltaAccelBias;
        Eigen::Vector3d deltaGyroBias;
        Eigen::Vector3d deltaGravity;
        Eigen::MatrixXd stateCovariance;
        Eigen::Matrix<double, 12, 12> noiseCovariance;
        Eigen::Matrix<double, 18, 18> stateJocobianMat;      // 雅可比矩阵（Phi矩阵）
        Eigen::Matrix<double, 18, 12> noiseCoefficientMat;   // 噪声系数矩阵（Gamma矩阵）
        Eigen::Matrix<double, 7, 7> observeNoiseCovariance;  // 
        ErrorState() : timeStamp(0), deltaQuaternion(1, 0, 0, 0), deltaTheta(0, Eigen::Vector3d(0, 0, 0)),
                       deltaPosition(Eigen::Vector3d::Zero()), deltaVelcity(Eigen::Vector3d::Zero()), deltaAccelBias(Eigen::Vector3d::Zero()),
                       deltaGyroBias(Eigen::Vector3d::Zero()), deltaGravity(Eigen::Vector3d::Zero()){}

    };

}; // namespace FBUSEKF

#endif
