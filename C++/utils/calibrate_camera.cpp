#include <iostream>
#include <sys/types.h>
#include <dirent.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;

// return 0: Error
int ScanFiles(std::vector<std::string> &fileList, std::string inputDirectory)
{
    inputDirectory = inputDirectory.append("/");

    DIR *p_dir;
    const char *str = inputDirectory.c_str();

    p_dir = opendir(str);
    if (p_dir == NULL)
    {
        std::cout << "Can't open :" << inputDirectory << endl;
        return 0;
    }

    struct dirent *p_dirent;

    while (p_dirent = readdir(p_dir))
    {
        string tmpFileName = p_dirent->d_name;
        if (tmpFileName == "." || tmpFileName == "..")
        {
            continue;
        }
        else
        {
            fileList.push_back(tmpFileName);
        }
    }
    closedir(p_dir);
    return fileList.size();
}

//检查文件(所有类型,包括目录和文件)是否存在
//返回1:存在 0:不存在
int IsFileExist(const char *path)
{
    return !access(path, F_OK);
}

void PrintUsage()
{
    std::cout << "Usage: "
              << "sudo ./pathTo/calibrate_camera [-d] [Path to save image] [-s] [Block size (unit: meter)]" << std::endl;
}

int main(int agec, char **argv)
{
    // Save Dir
    std::string imagePath;
    // Block size
    double blockSize = 0.1;

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
                imagePath = argv[i];
                paramIndex = 0;
                break;
            case 2:
                try
                {
                    blockSize = stod(argv[i]);
                }
                catch (const std::exception &)
                {
                    std::cout << "Block size shoule be a float number." << std::endl;
                    PrintUsage();
                    return 0;
                }
                break;
            default:
                paramIndex = 0;
                break;
            }

            if (paramIndex == 0)
            {
                if (!strcmp(argv[i], "-d"))
                    paramIndex = 1;
                else if (!strcmp(argv[i], "-s"))
                    paramIndex = 2;
                else
                    paramIndex = 0;
            }

            // std::cout << i << ", " << argv[i] << "," << paramIndex << std::endl;
        }
    }

    // 定义用来保存文件路径的容器
    std::vector<std::string> filelist;
    if (!ScanFiles(filelist, imagePath))
    {
        std::cout << "Please check the path of image.";
        PrintUsage();
    }
    // for (auto iter = filelist.begin(); iter != filelist.end(); iter++)
    //     std::cout << *iter << std::endl;


    // 定义用来保存导入的图片
    Mat image_in;
    // 定义用来保存旋转和平移矩阵的容器
    vector<Mat> rvecs, tvecs;
    // 定义相机矩阵，畸变矩阵
    Mat cameraMatrix;
    Mat distCoeffs;
    int flags = 0;
    // 定义保存图像二维角点的容器
    vector<Point2f> corners;
    // 定义保存图像三维角点的容器
    vector<vector<Point2f>> corners2;
    // 定义保存图像二维和三维角点的容器
    vector<Point3f> worldPoints;
    vector<vector<Point3f>> worldPoints2;

    //***********************生成一组object_points*************************
    for (int j = 0; j < 6; j++)
    {
        for (int k = 0; k < 8; k++)
        {
            worldPoints.push_back(Point3f(j * blockSize, k * blockSize, 0.0f));
        }
    }

    //***************************找角点××××××××××××××××××××××××××××××××
    std::cout << "Press Enter to next image." << std::endl;
    for (int i = 0; i < filelist.size(); i++)
    {
        //cout <<filelist[i]<<endl;
        // 一张张读入图片；
        image_in = imread(imagePath + filelist[i]);
        // 找图片的角点，参数分别为：
        // 输入图片，图片内角点数（不算棋盘格最外层的角点），输出角点，求解方式
        bool found = findChessboardCorners(image_in, Size(8, 6), corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        std::cout << "Corner num: " << corners.size() << std::endl;
        if (corners.size() > 0)
        {
            // 将找到的角点放入容器中；
            corners2.push_back(corners);
            // 世界坐标系的二维vector 放入三维vector
            worldPoints2.push_back(worldPoints);
        }
        else
        {
            std::cout << "This image is invalid. Corners cannot be detected. Please replace it with new image." << std::endl;
        }
        

        //画出角点
        drawChessboardCorners(image_in, Size(9, 6), corners, found);
        //显示图像
        imshow("calibrate", image_in);
        
        // 图像刷新等待时间
        while (1)
        {
            char key = static_cast<char>(cv::waitKey(1));
            if (key == 13) // 'Enter'
            {
                break;
            }
        }
    }

    // Calibrate
    cv::calibrateCamera(worldPoints2, corners2, image_in.size(), cameraMatrix, distCoeffs,
                    rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);

    //*************************************查看参数*****************************************
    std::cout << "************************************************************************************" << std::endl;
    std::cout << "************************************************************************************" << std::endl;
    std::cout << "Calibrated results: " << std::endl;
    std::cout << "************************************************************************************" << std::endl;
    // std::cout << "Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << std::endl;
    // std::cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << std::endl;
    // std::cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << std::endl;
    // std::cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << std::endl;
    std::cout << "Block size: " << blockSize << std::endl;
    std::cout << "Focal length: " << std::endl;
    std::cout << cameraMatrix.at<double>(0, 0) << std::endl;
    std::cout << cameraMatrix.at<double>(1, 1) << std::endl;
    std::cout << "Principal point: " << std::endl;
    std::cout << cameraMatrix.at<double>(0, 2) << std::endl;
    std::cout << cameraMatrix.at<double>(1, 2) << std::endl;
    std::cout << "Distortion coefficients: " << distCoeffs.rows << "x" << distCoeffs.cols << std::endl;
    for (int i = 0; i < distCoeffs.cols; i++)
    {
        std::cout << distCoeffs.at<double>(0, i) << std::endl;
    }

    // //*********************畸变矫正**************************
    // // 导入要矫正的图片
    // Mat test_image2 = imread("/camera_calibration/test_image.jpg");
    // Mat show_image;
    // undistort(test_image2, show_image, cameraMatrix, distCoeffs);
}
