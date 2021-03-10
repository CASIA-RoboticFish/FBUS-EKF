#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <functional>

#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>

#include <Eigen/Core>
#include <aruco/aruco.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <pangolin/pangolin.h>

#include "IMSEE-SDK/include/imrdata.h"
#include "IMSEE-SDK/include/imrsdk.h"
#include "IMSEE-SDK/include/svc_config.h"
#include "IMSEE-SDK/include/times.h"
#include "IMSEE-SDK/include/types.h"

#include "common.hpp"
#include "filter.hpp"
#include "vision.hpp"
#include "visualizer.hpp"
#include "yaml_eigen_converter.hpp"
#include "imu_data_generator.hpp"
#include "matrix_math.hpp"
#include "GlogHelper.hpp"

using namespace std;
std::string cameraConfigPath = "config/camerainfo.yml";
std::string paramConfigPath = "config/paramconfig.yml";
std::string markerSetupPath = "config/markersetup.yml";

//检查文件(所有类型,包括目录和文件)是否存在
//返回1:存在 0:不存在
int IsFileExist(const char *path)
{
    return !access(path, F_OK);
}

void PrintUsage()
{
    std::cout << "Usage: " << "sudo ./pathTo/underwater_aruco_ekf [-c/m/p/g] [specified param]" << std::endl;
    std::cout << "Param list: " << std::endl;
    std::cout << "  [-c] [camera_config_file_path]" << std::endl;
    std::cout << "  [-p] [param_config_file_path]" << std::endl;
    std::cout << "  [-m] [markermap_config_file_path]" << std::endl;
}

int ParseInputParam(int agec, char **argv)
{
    // Param identify
    if (agec < 2)
    {
        std::cout << "Parameters is two less!" << std::endl;
        PrintUsage();
        return 0;
    }
    else
    {
        int paramIndex = 0; // default: invalid param
        for (int i = 1; i < agec; i++)
        {
            switch (paramIndex)
            {
            case 1:
                if (!IsFileExist(argv[i]))
                {
                    std::cout << "Path is invalid." << std::endl;
                    PrintUsage();
                    return 0;
                }
                cameraConfigPath = argv[i];
                paramIndex = 0;
                break;
            case 2:
                if (!IsFileExist(argv[i]))
                {
                    std::cout << "Path is invalid." << std::endl;
                    PrintUsage();
                    return 0;
                }
                paramConfigPath = argv[i];
                paramIndex = 0;
                break;
            case 3:
                if (!IsFileExist(argv[i]))
                {
                    std::cout << "Path is invalid." << std::endl;
                    PrintUsage();
                    return 0;
                }
                markerSetupPath = argv[i];
                paramIndex = 0;
                break;
            default:
                paramIndex = 0;
                break;
            }

            if (paramIndex == 0)
            {
                if (!strcmp(argv[i], "-c"))
                    paramIndex = 1;
                else if (!strcmp(argv[i], "-p"))
                    paramIndex = 3;
                else if (!strcmp(argv[i], "-m"))
                    paramIndex = 4;
                else
                    paramIndex = 0;
            }
        }
    }
    return 1;
}

int main(int agec, char **argv)
{   
    if(!ParseInputParam(agec, argv))
        return 0;

#ifdef OPEN_DATA_RECORDING 
    std::ofstream ofs1;
    ofs1.open("data/imu.txt", ios::out);
    ofs1.close();

    std::ofstream ofs2;
    ofs2.open("data/image.txt", ios::out);
    ofs2.close();

    std::ofstream ofs3;
    ofs3.open("data/corners.txt", ios::out);
    ofs3.close();

    std::ofstream ofs4;
    ofs4.open("data/fusion.txt", ios::out);
    ofs4.close();

    if (!IsFileExist("data/image"))
    {
        int status;
        status = mkdir("data/image", S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO | S_IRWXU);
        if (status)
        {
            std::cout << "The path for saving data is invalid: " << status << std::endl;
            LOG(ERROR) << "The path for saving data is invalid: " << status;
        }
    }
#endif

    // Logging
    GLogHelper gh(argv[0]);

    // 设置相机驱动
    auto m_pSDK = new indem::CIMRSDK();
    indem::MRCONFIG config = {0};
    config.bSlam = false;
    config.imgResolution = indem::IMG_640;
    config.imgFrequency = 25;
    config.imuFrequency = 1000;
    m_pSDK->Init(config);
    LOG(INFO) << "Camera dirver has been activated ~~~";
    std::cout << "Camera dirver has been activated ~~~" << std::endl;

    // 读取相机参数
    YAML::Node cameraInfoNode = YAML::LoadFile(cameraConfigPath);
    FBUSEKF::CameraInfo leftCameraInfo(cameraInfoNode["Left Camera param"]);
    FBUSEKF::CameraInfo rightCameraInfo(cameraInfoNode["Right Camera param"]);
    FBUSEKF::IMUInfo imuInfo(cameraInfoNode["Imu param"]);
    LOG(INFO) << "Camera parameter has been load from " << cameraConfigPath;
    std::cout << "Camera parameter has been load from " << cameraConfigPath << std::endl;

    // 读取参数
    YAML::Node ParamConfigNode = YAML::LoadFile(paramConfigPath);
    FBUSEKF::VisionParam visionParam(ParamConfigNode["Vision Param"]);
    LOG(INFO) << "Vision parameter has been load from " << paramConfigPath;
    std::cout << "Vision parameter has been load from " << paramConfigPath << std::endl;

    FBUSEKF::RefractInfo refractInfo(ParamConfigNode["Refraction Param"]);
    LOG(INFO) << "Refract parameter has been load from " << paramConfigPath;
    std::cout << "Refract parameter has been load from " << paramConfigPath << std::endl;

    FBUSEKF::EkfParam ekfParam(ParamConfigNode["EKF Param"]);
    LOG(INFO) << "EKF parameter has been load from " << paramConfigPath;
    std::cout << "EKF parameter has been load from " << paramConfigPath << std::endl;

    // Read marker pose known in advance
    FBUSEKF::MarkerPoseServer markerPoseServer;
    YAML::Node markerGroupNode = YAML::LoadFile(markerSetupPath);
    YAML::Node markerNodeSet = markerGroupNode["Marker Group"];
    for(unsigned i=0;i<markerNodeSet.size();i++) {
        YAML::Node markerNode = markerNodeSet[i];
        FBUSEKF::MarkerPose markerPose;
        markerPose.markerID = markerNode["id"].as<double>();
        markerPose.positionAtG = markerNode["position"].as<Eigen::Vector3d>();
        Eigen::Matrix3d markerRotMat = markerNode["rotation"].as<Eigen::Matrix3d>();
        markerPose.quaternionM2G = Eigen::Quaterniond(markerRotMat);
        markerPoseServer.insert(std::pair<FBUSEKF::MarkerID, FBUSEKF::MarkerPose>(markerPose.markerID , markerPose));
    }


    // 创建滤波器，启动滤波器线程
    FBUSEKF::FILTER filter(leftCameraInfo, rightCameraInfo, imuInfo, ekfParam, markerPoseServer);
    filter.StartFilterThread();
    
    FBUSEKF::VISION vision(&filter, leftCameraInfo, rightCameraInfo, refractInfo, visionParam);
    vision.StartVisionThread();
    

    // 创建IMU数据发生器，将文件中记录的IMU数据，不断发送给滤波器线程
    // FBUSEKF::IMUGENTOR imuDataGenerator("data/data.txt");
    // imuDataGenerator.RegisterGetImuDataCallback(FBUSEKF::FILTER::InputIMUData, (void*)&filter); // 回调函数
    // std::function<void(FBUSEKF::IMUData)> imufun = std::bind(&FBUSEKF::FILTER::InputIMUDataFFFFF, filter, std::placeholders::_1);
    // imuDataGenerator.RegisterFunction(imufun);


#ifdef OPEN_VISUALIZER_THREAD
    // 启动可视化线程
    FBUSEKF::VISUALIZER pangolinVisualizer(&filter, &vision);
    pangolinVisualizer.StartVisualizerThread();
#endif


    // 图像回调函数设置
    std::queue<cv::Mat> image_queue;
    std::mutex mutex_image;
    int img_count = 0;
    m_pSDK->RegistImgCallback(
        [&img_count, &vision](double time, cv::Mat left, cv::Mat right) {
            if (!left.empty() && !right.empty())
            {
                FBUSEKF::ImageData imgdata;
                // cv::Mat img;
                // cv::hconcat(left, right, img);
                imgdata.timeStamp = time;
                imgdata.image0 = left;
                imgdata.image1 = right;
                vision.SetImageData(imgdata);
                vision.SetImageDataUpdated();
                ++img_count;
            }
        });
    LOG(INFO) << "Image callback has been load ~~~";
    std::cout << "Image callback has been load ~~~" << std::endl;

    // IMU回调函数设置
    int imu_count = 0;
    double g = imuInfo.g;
    m_pSDK->RegistModuleIMUCallback([&imu_count, &filter, &g](indem::ImuData imu) {
        FBUSEKF::IMUData imuData(imu.timestamp, imu.gyro[0]/180*M_PI,  imu.gyro[1]/180*M_PI, imu.gyro[2]/180*M_PI, imu.accel[0]*g, imu.accel[1]*g, imu.accel[2]*g);
        filter.SetImuData(imuData);
        filter.SetImuDataUpdated();
        // if(imu_count%10 == 0)
        //     std::cout << "IMU data receive: " << imu.gyro[0] << ", " << imu.gyro[1] << std::endl;
        ++imu_count;
    });
    LOG(INFO) << "IMU callback has been load ~~~";
    std::cout << "IMU callback has been load ~~~" << std::endl;
    LOG(INFO) << "FBUSEKF is ready ~~~";
    std::cout << "FBUSEKF is ready ~~~" << std::endl;
    // 等待线程结束
    filter.JoinFilterThread();
    vision.JoinVisionThread();
    // imuDataGenerator.JoinImuGenTorThread();
    

#ifdef OPEN_VISUALIZER_THREAD
    pangolinVisualizer.JoinVisualizerThread();
#endif

    // 释放
    delete m_pSDK;

    return 0;
}