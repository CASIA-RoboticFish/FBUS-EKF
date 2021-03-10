#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>

#include <iostream>
#include <queue>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "IMSEE-SDK/include/imrdata.h"
#include "IMSEE-SDK/include/imrsdk.h"
#include "IMSEE-SDK/include/logging.h"
#include "IMSEE-SDK/include/svc_config.h"
#include "IMSEE-SDK/include/times.h"
#include "IMSEE-SDK/include/types.h"

template <typename T>
void clear(std::queue<T> &q)
{
    std::queue<T> empty;
    swap(empty, q);
}

//检查文件(所有类型,包括目录和文件)是否存在
//返回1:存在 0:不存在
int IsFileExist(const char *path)
{
    return !access(path, F_OK);
}

//创建文件夹，返回0成功，返回-1不成功
int CreateFolder(const char *path)
{
    if (!IsFileExist(path))
    {
        int status;
        status = mkdir(path, S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO | S_IRWXU);
        if (status)
        {
            std::cout << "Path is invalid: " << status << std::endl;
        }
        return status;
    }
    return 0;
}

void PrintUsage()
{
    std::cout << "Usage: "
              << "sudo ./pathTo/take_photo [-d] [Path to save image] [-c] [left or right]" << std::endl;
}

int main(int agec, char **argv)
{
    // Save Dir
    std::string savePath = "Photo/Left";

    // Flag
    bool takePhoto = false;
    int cameraIndex = 0;

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
                savePath = "";
                savePath = savePath + argv[i] + "Photo/";
                paramIndex = 0;
                break;
            case 2:
                if (!strcmp(argv[i], "left"))
                {
                    cameraIndex = 0;
                }
                else if (!strcmp(argv[i], "right"))
                {
                    cameraIndex = 1;
                }
                else
                {
                    std::cout << "The param after -c must be left or right." << std::endl;
                    PrintUsage();
                    return 0;
                }
                paramIndex = 0;
                break;
            default:
                paramIndex = 0;
                break;
            }

            if (paramIndex == 0)
            {
                if (!strcmp(argv[i], "-d"))
                    paramIndex = 1;
                else if (!strcmp(argv[i], "-c"))
                    paramIndex = 2;
                else
                    paramIndex = 0;
            }

            // std::cout << i << ", " << argv[i] << "," << paramIndex << std::endl;
        }
    }

    // Create Folder
    std::cout << "Save path is " << savePath << std::endl;
    if (CreateFolder(savePath.c_str()))
    {
        PrintUsage();
        return 0;
    }

    if (cameraIndex == 0)
        savePath = savePath + "left";
    else if (cameraIndex == 1)
        savePath = savePath + "right";

    if (CreateFolder(savePath.c_str()))
    {
        PrintUsage();
        return 0;
    }

    // 设置相机驱动
    auto m_pSDK = new indem::CIMRSDK();
    indem::MRCONFIG config = {0};
    config.bSlam = false;
    config.imgResolution = indem::IMG_640;
    config.imgFrequency = 30;
    config.imuFrequency = 1000;
    m_pSDK->Init(config);

    // 图像回调函数设置
    int img_count = 0;
    std::queue<cv::Mat> image_queue;
    std::mutex mutex_image;
    m_pSDK->RegistImgCallback(
        [&img_count, &cameraIndex, &mutex_image, &image_queue](double time, cv::Mat left, cv::Mat right) {
            if (!left.empty() && !right.empty())
            {
                // cv::Mat img;
                // cv::hconcat(left, right, img);
                if (cameraIndex == 0)
                {
                    std::unique_lock<std::mutex> lock(mutex_image);
                    image_queue.push(left);
                }
                else
                {
                    std::unique_lock<std::mutex> lock(mutex_image);
                    image_queue.push(right);
                }

                ++img_count;
            }
        });

    std::cout << "1. Press ENTER to save photo." << std::endl;
    std::cout << "2. Press Q(q) to exit." << std::endl;
    while (1)
    {    
        if (!image_queue.empty())
        {
            std::unique_lock<std::mutex> lock(mutex_image);
            cv::imshow("image", image_queue.front());
            if (takePhoto)
            {
                cv::imwrite(savePath + "/" + std::to_string(img_count) + ".jpg", image_queue.front());
                std::cout << "Take photo: " << img_count << ". Succeed."<<std::endl;
                takePhoto = false;
            }
            clear(image_queue);
        }
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q')
        { // ESC/Q
            break;
        }
        else if (key == 13) // 'Enter'
        {
            takePhoto = true;
        }
    }
}