#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>


int main(int agec, char **argv)
{
    cv::Mat cameraMat1(3,3,CV_32F);
    cameraMat1.at<float>(0,0) = 246.134;
    cameraMat1.at<float>(0,1) = 0;
    cameraMat1.at<float>(0,2) = 325.504;
    cameraMat1.at<float>(1,0) = 0;
    cameraMat1.at<float>(1,1) = 246.265;
    cameraMat1.at<float>(1,2) = 178.694;
    cameraMat1.at<float>(2,0) = 0;
    cameraMat1.at<float>(2,1) = 0;
    cameraMat1.at<float>(2,2) = 1;


    cv::Mat distortion1(4,1,CV_32F);
    distortion1.at<float>(0,0) = 0.584804;
    distortion1.at<float>(0,1) = 0.158016;
    distortion1.at<float>(0,2) = -0.5657;
    distortion1.at<float>(0,3) = 0.272636;


    float x = 0.6666;
    float y = 0.2961;
    std::vector<cv::Point2f> undistortedCorners;
    undistortedCorners.push_back(cv::Point2f(x, y));

    std::vector<cv::Point2f> distortedCorners;

    cv::fisheye::distortPoints(undistortedCorners, distortedCorners, cameraMat1, distortion1);

    

    float r = std::sqrt(x*x + y*y);
    float theta = atan(r);
    float theta2 = theta * theta;
    float theta4 = theta2 * theta2;
    float theta6 = theta2 * theta4;
    float theta8 = theta4 * theta4;

    float thetaD = theta * (1 + distortion1.at<float>(0,0)*theta2 + distortion1.at<float>(0,1)*theta4 \ 
                              + distortion1.at<float>(0,2)*theta6 + distortion1.at<float>(0,3)*theta8);
    
    float xd = thetaD * x / r;
    float yd = thetaD * y / r;

    float u = cameraMat1.at<float>(0,0) * xd + cameraMat1.at<float>(0,2);
    float v = cameraMat1.at<float>(1,1) * yd + cameraMat1.at<float>(1,2);

    std::cout << "my cal: " << u << ", " << v << std::endl;
    std::cout << "opencv: " << distortedCorners[0].x << ", " << distortedCorners[0].y << std::endl;

    distortedCorners.push_back(cv::Point2f(u, v));

    std::vector<cv::Point2f> undistortedCorners_new;
    cv::fisheye::undistortPoints(distortedCorners, undistortedCorners_new, cameraMat1, distortion1);

    std::cout << undistortedCorners_new[0].x << ", " << undistortedCorners_new[0].y << std::endl;
    std::cout << undistortedCorners_new[1].x << ", " << undistortedCorners_new[1].y << std::endl;
    return 0;

}