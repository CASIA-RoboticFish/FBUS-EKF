#ifndef IMU_DATA_GENERATOR_HPP
#define IMU_DATA_GENERATOR_HPP

#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <functional>

#include "common.hpp"

namespace FBUSEKF
{

typedef void (*GetImuDataCallbackFun)(FBUSEKF::IMUData imudata, void* pObject);
typedef std::function<void(FBUSEKF::IMUData)> GetImuCallbackFunction;


class IMUGENTOR
{
public:
    IMUGENTOR():imuGenTorThread_(std::bind(&IMUGENTOR::ReadImuDataFromTxt,this)),isCallbackFunInit_(false){}
    IMUGENTOR(std::string filename): txtFileName_(filename),imuGenTorThread_(std::bind(&IMUGENTOR::ReadImuDataFromTxt,this)),isCallbackFunInit_(false){}
    ~IMUGENTOR(){}

    void ReadImuDataFromTxt()
    {
        std::ifstream inFile;
        inFile.open(txtFileName_);
        std::string oneline;
        int count = 0;
        while(std::getline(inFile,oneline))
        {
            if(!isCallbackFunInit_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }
            std::istringstream splitLine(oneline);
            splitLine >> timeStamp >> gx >> gy >> gz >> ax >> ay >> az;       
            count++;
            // std::cout << "IMUdata" << std::endl;
            FBUSEKF::IMUData imuData(timeStamp, gx,  gy, gz, ax, ay, az);
            fGetImuData(imuData, pObject_);
            // fGetImuFFFFFF(imuData);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "There is no any imu data." << std::endl;
    }

    void RegisterGetImuDataCallback(GetImuDataCallbackFun fFun, void* pObject)
    {
        fGetImuData = fFun;
        pObject_ = pObject;
        isCallbackFunInit_ = true;
    }

    // void RegisterFunction(GetImuCallbackFunction fun)
    // {
    //     fGetImuFFFFFF = fun;
    //     isCallbackFunInit_ = true;
    // }

    void JoinImuGenTorThread()
    {
        imuGenTorThread_.join();
    }

private:
    std::string txtFileName_;
    double timeStamp, ax, ay, az, gx, gy, gz;
    GetImuDataCallbackFun fGetImuData;
    GetImuCallbackFunction fGetImuFFFFFF;

    std::thread imuGenTorThread_;
    void* pObject_;
    bool isCallbackFunInit_;
};

};

#endif