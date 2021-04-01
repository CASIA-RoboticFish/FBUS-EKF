# FBUS-EKF

The FBUS-EKF package is the code implementation of the proposed fiducial-based underwater stereo visual-inertial localization method, which addresses the refraction effect in underwater localization. Specifically, this method detects the artificial marker from images and utilize the refractive camera model to compute the precise marker pose, finally the inertial data is fused with the marker pose to obtain the robust camera pose. The software takes stereo images, IMU data, and a known marker map, then generates real-time 6DOF pose estimation of the IMU frame.

The code implementation includes  Matlab and C++ versions. The Matlab code is tested under the platform of Matlab 2017b. The Matlab code only focuses on the fusion of marker pose and inertial data other than the visual processing, which takes in the marker pose and IMU data, and outputs the 6DOF pose estimation of the IMU frame. The C++ version is tested on Ubuntu 18.04, which provides the complete system implementation from visual processing to filter fusion.



## Usage

#### Matlab Verison

The core m-file is FBUS-EKF.m in Matlab implementation, just run it on the Matlab environment. Besides, the dataset folder contains the data record during land and underwater environment.

#### C++ Verision

At present, this software can only run on the sensor suite from Indemind company. The product link is http://indemind.cn/module. A more general version will be put up later.

##### Dependencies

- Eigen3

- OpenCV 3.4.3: https://github.com/opencv/opencv

- ArUco: https://www.uco.es/investiga/grupos/ava/node/26

- yaml-cpp: https://github.com/jbeder/yaml-cpp

- glog: https://github.com/google/glog

- Pangolin: https://github.com/stevenlovegrove/Pangolin

- IMSEE-SDK: https://github.com/INDEMIND/IMSEE-SDK



## License

FBUS-EKF is licensed generally under a permissive GPL license.



## Contact

If you have any problem, or you have some suggestions for this code, please contact Pengfei Zhang by [zhangpengfei2017@ia.ac.cn](mailto:315261982@qq.com), thank you very much!



























