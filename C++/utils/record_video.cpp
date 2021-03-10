#include <termio.h>
#include <stdio.h>
#include <unistd.h>

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

int scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings = stored_settings;            //
    new_settings.c_lflag &= (~ICANON);         //
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(STDIN_FILENO, &stored_settings); //获得stdin 输入
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings); //
    in = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &stored_settings);
    return in;
}

int main(int agec, char **argv)
{
    // 设置相机驱动
    auto m_pSDK = new indem::CIMRSDK();
    indem::MRCONFIG config = {0};
    config.bSlam = false;
    config.imgResolution = indem::IMG_640;
    config.imgFrequency = 30;
    config.imuFrequency = 1000;
    m_pSDK->Init(config);

    // std::string videoname = "test.avi";
    int isColor = 0;
    int fps = 25;
    int frameWidth = 1280;
    int frameHeight = 400;
    cv::VideoWriter aviWriter("test.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight), isColor);

    // 图像回调函数设置
    int img_count = 0;
    std::queue<cv::Mat> image_queue;
    std::mutex mutex_image;
    m_pSDK->RegistImgCallback(
        [&img_count, &aviWriter, &image_queue, &mutex_image](double time, cv::Mat left, cv::Mat right) {
            if (!left.empty() && !right.empty())
            {
                cv::Mat img;
                cv::hconcat(left, right, img);
                aviWriter.write(img);
                ++img_count;
                {
                    std::unique_lock<std::mutex> lock(mutex_image);
                    image_queue.push(img);
                }
            }
        });

    while (1)
    {
        // int key = scanKeyboard();
        // std::cout << printf(":%d\r\n", key);
        // if (key == 's')
        //     break;
        std::cout << "Recording video. Press Q(q) to exit and save." << std::endl;
        if (!image_queue.empty())
        {
            std::unique_lock<std::mutex> lock(mutex_image);
            cv::imshow("image", image_queue.front());
            clear(image_queue);
        }
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q')
        { // ESC/Q
            break;
        }

    }
}