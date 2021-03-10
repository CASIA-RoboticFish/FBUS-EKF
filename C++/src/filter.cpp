#include "filter.hpp"

/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::InputIMUData(FBUSEKF::IMUData imudata, void *pObject)
{
    std::cout << "Get IMU data at " << std::fixed << std::setprecision(0) << imudata.timeStamp << std::endl;
    FBUSEKF::FILTER *mySelf = (FBUSEKF::FILTER *)pObject;
    mySelf->SetImuData(imudata);

    // 这后面可以写对于IMU数据的处理程序
    //

    mySelf->SetImuDataUpdated();
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::SetImuData(FBUSEKF::IMUData imudata)
{
    std::unique_lock<std::mutex> lck(this->imuMutex);

#ifdef OPEN_DATA_RECORDING
    std::ofstream ofs;
    ofs.open("data/imu.txt", std::ios::app);
    ofs << imudata.timeStamp << " " << imudata.accel[0]  << " " << imudata.accel[1]  << " " << imudata.accel[2]
         << " " << imudata.gyro[0]  << " " << imudata.gyro[1]  << " " << imudata.gyro[2] << std::endl;
    ofs.close();
#endif

    if(this->imuMeasuementBuffer_.size()==0)
    {
        this->imuMeasuementBuffer_.push_back(imudata);
    }
    else
    {
        FBUSEKF::IMUData imudataFilter;
        double filter_coeff = 0.1;
        imudataFilter.timeStamp = imudata.timeStamp;
        imudataFilter.accel = this->imuMeasuementBuffer_.back().accel * (1-filter_coeff) + imudata.accel * filter_coeff;
        imudataFilter.gyro = this->imuMeasuementBuffer_.back().gyro * (1-filter_coeff) + imudata.gyro * filter_coeff;
        this->imuMeasuementBuffer_.push_back(imudataFilter);
    }

    if (this->imuMeasuementBuffer_.size() > IMU_BUFFER_MAX_SIZE)
    {
        this->ClearImuBuffer(this->imuMeasuementBuffer_.begin(), this->imuMeasuementBuffer_.begin() + 500);
        LOG(INFO) << "Clear IMU buffer at " << this->imuMeasuementBuffer_.end()->timeStamp;
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::SetDetectionResult(FBUSEKF::DetectionResultList &result)
{
    std::unique_lock<std::mutex> lck(this->imgMutex);
    this->detectionResult_ = result;
}
/****
 *
 *
 *
 ****/
Eigen::Matrix4d FBUSEKF::FILTER::GetCameraPose() // 主要给可视化线程用的，Pangolin
{
    std::unique_lock<std::mutex> lck1(this->visualMutex);
    Eigen::Matrix4d cameraPose = Eigen::Matrix4d::Zero();
    if (this->isInitializePose_)
    {
        cameraPose.block<3, 3>(0, 0) = this->sysNominalState_.rotmatI2G;
        cameraPose.block<3, 1>(0, 3) = this->sysNominalState_.positionAtG;
        cameraPose(3, 3) = 1;
    }
    return cameraPose;
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::GetStaticMarkerPose(std::vector<Eigen::Matrix4d> &markerPoses)
{
    std::unique_lock<std::mutex> lck1(this->visualMutex);
    if (this->isInitializePose_)
    {
        for (auto iter = this->markerPoseServer_.begin(); iter != this->markerPoseServer_.end(); iter++)
        {
            Eigen::Matrix4d markerPose = Eigen::Matrix4d::Zero();
            markerPose.block<3, 3>(0, 0) = iter->second.quaternionM2G.toRotationMatrix();
            markerPose.block<3, 1>(0, 3) = iter->second.positionAtG;
            markerPose(3, 3) = 1;
            markerPoses.push_back(markerPose);
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::GetDynamicMarkerPose(std::vector<Eigen::Matrix4d> &markerPoses)
{
    std::unique_lock<std::mutex> lck1(this->visualMutex);
    if (this->isInitializePose_)
    {
        for (auto iter = this->markerPoseServer_.begin(); iter != this->markerPoseServer_.end(); iter++)
        {
            Eigen::Matrix4d markerPose = Eigen::Matrix4d::Zero();
            markerPose.block<3, 3>(0, 0) = iter->second.quaternionM2G.toRotationMatrix();
            markerPose.block<3, 1>(0, 3) = iter->second.positionAtG;
            markerPose(3, 3) = 1;
            markerPoses.push_back(markerPose);
        }
    }
}
/****
 *
 *
 *
 ****/
Eigen::Matrix4d FBUSEKF::FILTER::GetVisualPose()
{
    std::unique_lock<std::mutex> lck1(this->visualMutex);
    Eigen::Matrix4d cameraPose = Eigen::Matrix4d::Zero();
    if (this->isInitializePose_)
    {
        cameraPose.block<3, 3>(0, 0) = this->sysNominalState_.quaternionOnlyVisual.toRotationMatrix();
        cameraPose.block<3, 1>(0, 3) = this->sysNominalState_.positionOnlyVisual;
        cameraPose(3, 3) = 1;
    }
    return cameraPose;
}

void FBUSEKF::FILTER::GetVisualizeInfo(Eigen::Matrix4d& cameraPose, Eigen::Matrix4d& visualPose, std::vector<Eigen::Matrix4d>& staticMarkerPoses, std::vector<Eigen::Matrix4d>& dynamicMarkerPoses)
{
    std::unique_lock<std::mutex> lck1(this->visualMutex);
    cameraPose = Eigen::Matrix4d::Zero();
    visualPose = Eigen::Matrix4d::Zero();
    if (this->isInitializePose_)
    {

        cameraPose.block<3, 3>(0, 0) = this->sysNominalState_.rotmatI2G;
        cameraPose.block<3, 1>(0, 3) = this->sysNominalState_.positionAtG;
        cameraPose(3, 3) = 1;

        visualPose.block<3, 3>(0, 0) = this->sysNominalState_.quaternionOnlyVisual.toRotationMatrix();
        visualPose.block<3, 1>(0, 3) = this->sysNominalState_.positionOnlyVisual;
        visualPose(3, 3) = 1;

        for (auto iter = this->markerPoseServer_.begin(); iter != this->markerPoseServer_.end(); iter++)
        {
            Eigen::Matrix4d markerPose = Eigen::Matrix4d::Zero();
            markerPose.block<3, 3>(0, 0) = iter->second.quaternionM2G.toRotationMatrix();
            markerPose.block<3, 1>(0, 3) = iter->second.positionAtG;
            markerPose(3, 3) = 1;
            staticMarkerPoses.push_back(markerPose);
        }

        for (auto iter = this->markerPoseServer_.begin(); iter != this->markerPoseServer_.end(); iter++)
        {
            Eigen::Matrix4d markerPose = Eigen::Matrix4d::Zero();
            markerPose.block<3, 3>(0, 0) = iter->second.quaternionM2G.toRotationMatrix();
            markerPose.block<3, 1>(0, 3) = iter->second.positionAtG;
            markerPose(3, 3) = 1;
            dynamicMarkerPoses.push_back(markerPose);
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::StartFilterThread()
{
    this->filterThread_ = std::thread(std::bind(&FBUSEKF::FILTER::FilterThreadFunction, this));
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::FilterThreadFunction()
{
    LOG(INFO) << "Filter thread has been started ~~~" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1000ms, 收集IMU数据, 进行初始化

    // 初始化系统状态
    this->InitializeGravityAndBias();

    // 线程主循环
    while (1)
    {
        std::unique_lock<std::mutex> lck(this->ekfMutex);
        this->ekfCondVar.wait(lck); // 每次收到视觉检测结果时，本线程就会被唤醒
        std::unique_lock<std::mutex> lck1(this->visualMutex);
        LOG_EVERY_N(INFO, 1000) << "Carryout EKF update at " << std::fixed << std::setprecision(6) << this->imuMeasuementBuffer_.begin()->timeStamp << std::endl;

        // 初始化IMU Pose，之所以不和重力矢量一起初始化是因为，有可能刚上电的时候，并没有检测到Marker，所以一定要检测到Marker时才初始化IMU Pose
        if (!this->isInitializePose_)
        {
            if (this->InitializePose())
            {
                this->isInitializePose_ = true;

                LOG(INFO) << "System init success: ";
                LOG(INFO) << "Iint position: " << this->sysNominalState_.positionAtG[0] << ", " << this->sysNominalState_.positionAtG[1] << ", " << this->sysNominalState_.positionAtG[2];
                LOG(INFO) << "Iint velocity: " << this->sysNominalState_.velocityAtG[0] << ", " << this->sysNominalState_.velocityAtG[1] << ", " << this->sysNominalState_.velocityAtG[2];
                LOG(INFO) << "Iint quaternion: " << this->sysNominalState_.quaternionI2G.w() << ", " << this->sysNominalState_.quaternionI2G.x()
                          << ", " << this->sysNominalState_.quaternionI2G.y() << ", " << this->sysNominalState_.quaternionI2G.z();
                LOG(INFO) << "Iint accel bias: " << this->sysNominalState_.accelBias[0] << ", " << this->sysNominalState_.accelBias[1] << ", " << this->sysNominalState_.accelBias[2];
                LOG(INFO) << "Iint gyro bias: " << this->sysNominalState_.gyroBias[0] << ", " << this->sysNominalState_.gyroBias[1] << ", " << this->sysNominalState_.gyroBias[2];
                LOG(INFO) << "System covariance:";
                LOG(INFO) << this->sysErrorState_.stateCovariance;
                continue;
            }
            else
                continue;
        }

        // Reset
        this->ResetSystemState();

        // IMU时间更新
        this->BatchImuProcessing();

        // 视觉量测更新
        this->ObservationUpdate();


#ifdef OPEN_DATA_RECORDING
    std::ofstream ofs;
    ofs.open("data/fusion.txt", std::ios::app);
    ofs << this->sysNominalState_.timeStamp << " " << this->sysNominalState_.positionAtG[0]  << " " << this->sysNominalState_.positionAtG[1]  << " " << this->sysNominalState_.positionAtG[2]
        << " " << this->sysNominalState_.quaternionI2G.w() << " " << this->sysNominalState_.quaternionI2G.x() << " " << this->sysNominalState_.quaternionI2G.y()  << " " << this->sysNominalState_.quaternionI2G.z()
        << " " << this->sysNominalState_.velocityAtG[0] << " " << this->sysNominalState_.velocityAtG[1] << " " << this->sysNominalState_.velocityAtG[2] 
        << " " << this->sysNominalState_.accelBias[0] << " " << this->sysNominalState_.accelBias[1] << " " << this->sysNominalState_.accelBias[2]
        << " " << this->sysNominalState_.gyroBias[0] << " " << this->sysNominalState_.gyroBias[1] << " " << this->sysNominalState_.gyroBias[2] 
        << std::endl;
    ofs.close();
#endif
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::InitializeGravityAndBias()
{
    std::unique_lock<std::mutex> lck(this->imuMutex);
    if (this->imuMeasuementBuffer_.size() > 0)
    {
        this->sysNominalState_.timeStamp = this->imuMeasuementBuffer_.end()->timeStamp;
        this->sysErrorState_.timeStamp = this->imuMeasuementBuffer_.end()->timeStamp;

        // 求加速度计和陀螺仪数据的平均值
        int dataNum = this->imuMeasuementBuffer_.size();
        Eigen::Vector3d accelMean = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyroMean = Eigen::Vector3d::Zero();
        for (auto iter = this->imuMeasuementBuffer_.begin(); iter != this->imuMeasuementBuffer_.end(); iter++)
        {
            accelMean = accelMean + iter->accel;
            gyroMean = gyroMean + iter->gyro;
        }

        // 初始化
        this->sysNominalState_.gyroBias = gyroMean / dataNum;
        this->sysNominalState_.gravityAtG = Eigen::Vector3d(0.0, 0.0, -(accelMean / dataNum).norm());

        // 清除使用过的IMU数据
        this->ClearImuBuffer(this->imuMeasuementBuffer_.begin(), this->imuMeasuementBuffer_.end());

        LOG(INFO) << "The gravity and gyros bias has been initialized at " << this->sysNominalState_.timeStamp;
        // LOG(INFO) << "Init gyro bias: " << this->sysNominalState_.gyroBias[0] << ", " << this->sysNominalState_.gyroBias[1] << ", " << this->sysNominalState_.gyroBias[2];
        // LOG(INFO) << "Init gravity: " << this->sysNominalState_.gravityAtG[0] << ", " << this->sysNominalState_.gravityAtG[1] << ", " << this->sysNominalState_.gravityAtG[2];
    }
}
/****
 *
 *
 *
 ****/
bool FBUSEKF::FILTER::InitializePose()
{
    std::unique_lock<std::mutex> lck1(this->imuMutex);
    std::unique_lock<std::mutex> lck2(this->imgMutex);
    if (this->imuMeasuementBuffer_.size() > 0 && this->detectionResult_.size() > 0)
    {
        // 找出视觉检测结果之前的IMU数据
        double detectionTime = this->detectionResult_[0].timeStamp;
        int imuCnt = 0;
        for (const auto &imuData : this->imuMeasuementBuffer_)
        {
            if (imuData.timeStamp > detectionTime)
                break;
            imuCnt++;
        }

        // 如果在视觉检测结果之前没有IMU数据，报错，说明初始化不成功
        if (imuCnt == 0)
        {
            LOG(ERROR) << "Initialize pose failed. The detection timestamp is earlier than IMU timestamp. There is not any available IMU data.";
            return false;
        }

        // 利用视觉检测结果之前的一部分数据，初始化IMU系的姿态
        int startIdx = imuCnt - IMUDATA_NUM_FOR_INIT_POSE;
        if (startIdx < 0)
            startIdx = 0;

        // 计算加速度矢量（我们假设第一次检测到Marker的时刻，系统是基本静止的）
        Eigen::Vector3d accelMean = Eigen::Vector3d::Zero();
        for (int i = startIdx; i < imuCnt; i++)
        {
            accelMean = accelMean + this->imuMeasuementBuffer_[i].accel;
        }
        accelMean = accelMean / (imuCnt - startIdx);

        // 初始化系统状态
        // Read marker pose in LIB
        int count = 0;
        int min_dist_id = 0;
        double min_dist = 10;
        for (const auto &detectmarker : this->detectionResult_)
        {
            double distance = detectmarker.positionAtCL.norm();
            if(distance < min_dist)
            {
                min_dist = distance;
                min_dist_id = count;
            }
            count++;
        }
        
        if(min_dist > this->ekfParam_.fcpMarkerMaxDist)
        {
            LOG(WARNING) << "Initialize pose failed. This marker is out the detection range.";
            return false;
        }
        
        FBUSEKF::DetectionResult nearest_marker = this->detectionResult_[min_dist_id];

        int markerID = nearest_marker.markerID;
        // 确认系统状态中是否包含该Marker，如果不包含，则不需要量测更新，就直接return
        auto markerIter = this->markerPoseServer_.find(markerID);
        if(markerIter == this->markerPoseServer_.end())
        {
            LOG(WARNING) << "Initialize pose failed. This marker is not in marker server.";
            return false;
        }

        
        Eigen::Quaterniond Q_M_G = markerIter->second.quaternionM2G;
        Eigen::Vector3d P_M_G = markerIter->second.positionAtG;
 
        // Read detection result
        Eigen::Quaterniond Q_M_L = nearest_marker.quaternionM2CL;
        Eigen::Vector3d P_M_L = nearest_marker.positionAtCL;

        // Read camera parameter
        Eigen::Matrix3d R_I_L = this->cameraInfoList_[0].T_SC.block<3, 3>(0, 0);
        Eigen::Quaterniond Q_I_L(R_I_L);
        Eigen::Vector3d P_L_I = this->cameraInfoList_[0].T_SC.block<3, 1>(0, 3);
        Eigen::Vector3d P_I_L = -R_I_L.transpose() * P_L_I;

        // Time
        this->sysNominalState_.timeStamp = detectionTime;
        this->sysErrorState_.timeStamp = detectionTime;

        // IMU Orientation
        Eigen::Quaterniond Q_I_G = Q_M_G * Q_M_L.conjugate() * Q_I_L;
        this->sysNominalState_.quaternionI2G = Q_I_G;
        this->sysNominalState_.rotmatI2G = Q_I_G.toRotationMatrix();

        // IMU Position
        this->sysNominalState_.positionAtG = P_M_G - this->sysNominalState_.rotmatI2G*P_I_L - this->sysNominalState_.rotmatI2G*R_I_L.transpose()*P_M_L;

        // Gravity
        this->sysNominalState_.gravityAtG = Eigen::Vector3d(9.8, 0, 0);

        // 清除用过的IMU数据
        this->ClearImuBuffer(this->imuMeasuementBuffer_.begin(), this->imuMeasuementBuffer_.begin() + imuCnt);

        LOG(INFO) << "The IMU pose and attitude has been initialized at " << this->sysNominalState_.timeStamp;
        LOG(INFO) << "Iint position: " << this->sysNominalState_.positionAtG[0] << ", " << this->sysNominalState_.positionAtG[1] << ", " << this->sysNominalState_.positionAtG[2];
        LOG(INFO) << "Iint quaternion: " << this->sysNominalState_.quaternionI2G.w() << ", " << this->sysNominalState_.quaternionI2G.x()
                  << ", " << this->sysNominalState_.quaternionI2G.y() << ", " << this->sysNominalState_.quaternionI2G.z();
        return true;
    }
    return false;
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::ResetSystemState()
{
    std::unique_lock<std::mutex> lck(this->imgMutex);

    if (this->detectionResult_.size() > 0)
    {
        Eigen::Matrix3d R_I_L = this->cameraInfoList_[0].T_SC.block<3, 3>(0, 0);
        Eigen::Quaterniond Q_I_L(R_I_L);
        Eigen::Vector3d P_L_I = this->cameraInfoList_[0].T_SC.block<3, 1>(0, 3);
        Eigen::Vector3d P_I_L = -R_I_L.transpose() * P_L_I;
        // Todo: 
        // 这一段代码写的有问题, 我只用了第一个检测到的marker来重置相机位姿

        int count = 0;
        int min_dist_id = 0;
        double min_dist = 10;
        for (const auto &detectmarker : this->detectionResult_)
        {
            double distance = detectmarker.positionAtCL.norm();
            if(distance < min_dist)
            {
                min_dist = distance;
                min_dist_id = count;
            }
            count++;
        }
        
        if(min_dist > this->ekfParam_.fcpMarkerMaxDist)
        {
            LOG(WARNING) << "Reset pose failed. This marker is out the detection range.";
            return;
        }
        
        FBUSEKF::DetectionResult nearest_marker = this->detectionResult_[min_dist_id];

        int markerID = nearest_marker.markerID;
        // 确认系统状态中是否包含该Marker，如果不包含，则不需要量测更新，就直接return
        auto markerIter = this->markerPoseServer_.find(markerID);
        if(markerIter == this->markerPoseServer_.end())
        {
            LOG(WARNING) << "Reset pose failed. This marker is not in marker server.";
            return;
        }

        Eigen::Quaterniond Q_M_G = markerIter->second.quaternionM2G;
        Eigen::Vector3d P_M_G = markerIter->second.positionAtG;

        // 计算相机姿态（仅根据相机观测数据和Marker在世界系中的姿态）
        Eigen::Quaterniond Q_M_L = nearest_marker.quaternionM2CL;
        this->sysNominalState_.quaternionOnlyVisual = Q_M_G * Q_M_L.conjugate() * Q_I_L;
        Eigen::Matrix3d R_I_G = this->sysNominalState_.quaternionOnlyVisual.toRotationMatrix();

        // 计算相机位置（仅根据相机观测数据和Marker在世界系中的位置）
        Eigen::Vector3d P_M_L = nearest_marker.positionAtCL;
        this->sysNominalState_.positionOnlyVisual = -R_I_G * R_I_L.transpose() * P_M_L + P_M_G - R_I_G * P_I_L;

        // 如果当前检测结果相距上一次系统量测更新的状态间隔太久（超过0.1秒），且系统已经完成初始化，那么就选择重置系统状态
        if (this->detectionResult_[0].timeStamp - this->sysNominalState_.timeStamp > 0.1 && this->isInitializePose_)
        {
            LOG(INFO) << "Reset system state at time: " << this->sysNominalState_.timeStamp;

            this->sysNominalState_.timeStamp = nearest_marker.timeStamp;
            this->sysNominalState_.quaternionI2G = this->sysNominalState_.quaternionOnlyVisual;
            this->sysNominalState_.positionAtG = this->sysNominalState_.positionOnlyVisual;
            this->sysNominalState_.velocityAtG = Eigen::Vector3d::Zero();
            this->sysNominalState_.accelBias = Eigen::Vector3d::Zero();
            this->sysNominalState_.gyroBias = Eigen::Vector3d::Zero();

            this->sysErrorState_.timeStamp = nearest_marker.timeStamp;
        }
            
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::BatchImuProcessing()
{
    std::unique_lock<std::mutex> lck1(this->imuMutex);
    std::unique_lock<std::mutex> lck2(this->imgMutex);

    if (this->imuMeasuementBuffer_.size() > 0)
    {
        double startTime = this->sysNominalState_.timeStamp;
        double endTime = this->detectionResult_[0].timeStamp;
        int imuCnt = 0;
        for (const auto &imuData : this->imuMeasuementBuffer_)
        {
            // 确定IMU数据的时间范围
            if (imuData.timeStamp < startTime)
            {
                imuCnt++;
                continue;
            }
            if (imuData.timeStamp > endTime)
                break;
            imuCnt++;

            Eigen::Vector3d accel = imuData.accel;
            Eigen::Vector3d gyro = imuData.gyro;
            double dt = imuData.timeStamp - this->sysNominalState_.timeStamp;

            // 更新误差状态协方差矩阵, 先更新协方差矩阵，以免系统状态被修改
            this->UpdateCovariance(dt, accel, gyro);

            // 更新Nominal状态
            this->UpdateNominalState(dt, accel, gyro);

            // 修改Nominal状态的时间戳
            this->sysNominalState_.timeStamp = imuData.timeStamp;
        }

        // 清空用过的IMU数据
        this->ClearImuBuffer(this->imuMeasuementBuffer_.begin(), this->imuMeasuementBuffer_.begin() + imuCnt);

        //
        LOG(INFO) << "IMU batch processing at time: " << this->sysNominalState_.timeStamp;
        LOG(INFO) << "IMU Position: " << this->sysNominalState_.positionAtG[0] << ", " << this->sysNominalState_.positionAtG[1] << ", " << this->sysNominalState_.positionAtG[2];
        LOG(INFO) << "IMU Quaternion: " << this->sysNominalState_.quaternionI2G.w() << ", " << this->sysNominalState_.quaternionI2G.x() << ", "
                  << this->sysNominalState_.quaternionI2G.y() << ", " << this->sysNominalState_.quaternionI2G.z();
        LOG(INFO) << "IMU RotateMatrix: " << this->sysNominalState_.rotmatI2G;

        // LOG(INFO) << this->sysErrorState_.stateCovariance;
    }
}

void FBUSEKF::FILTER::UpdateNominalState(double dtime, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro)
{
    // 时间间隔计算
    double deltaT = dtime;

    // 姿态更新
    Eigen::Vector3d angularVel = gyro - this->sysNominalState_.gyroBias;
    double angularVelNorm = angularVel.norm();
    Eigen::Quaterniond quaternionI2GHalfTime; // 用于后面的速度更新的龙格库塔算法
    Eigen::Matrix3d rotmatI2GBeginTime = this->sysNominalState_.quaternionI2G.toRotationMatrix();

    if (angularVelNorm > 10e-5)
    {
        Eigen::AngleAxisd deltaAngleHalf(angularVelNorm * deltaT / 2, angularVel / angularVelNorm);
        quaternionI2GHalfTime = this->sysNominalState_.quaternionI2G * Eigen::Quaterniond(deltaAngleHalf);
        quaternionI2GHalfTime.normalize();
        Eigen::AngleAxisd deltaAngle(angularVelNorm * deltaT, angularVel / angularVelNorm);
        this->sysNominalState_.quaternionI2G = this->sysNominalState_.quaternionI2G * Eigen::Quaterniond(deltaAngle);
        this->sysNominalState_.quaternionI2G.normalize();
    }
    else
    {
        quaternionI2GHalfTime = this->sysNominalState_.quaternionI2G *
                                Eigen::Quaterniond(1, 0.5 * deltaT * angularVel(0) / 2, 0.5 * deltaT * angularVel(1) / 2, 0.5 * deltaT * angularVel(2) / 2);
        quaternionI2GHalfTime.normalize();
        this->sysNominalState_.quaternionI2G = this->sysNominalState_.quaternionI2G *
                                               Eigen::Quaterniond(1, 0.5 * deltaT * angularVel(0), 0.5 * deltaT * angularVel(1), 0.5 * deltaT * angularVel(2));
        this->sysNominalState_.quaternionI2G.normalize();
    }
    Eigen::Matrix3d rotmatI2GHalfTime = quaternionI2GHalfTime.toRotationMatrix();

    this->sysNominalState_.rotmatI2G = this->sysNominalState_.quaternionI2G.toRotationMatrix();
    this->sysNominalState_.eulerAngle = this->sysNominalState_.rotmatI2G.eulerAngles(2, 1, 0); // Z-Y-X顺序

    // 速度更新, 四阶龙格库塔算法
    Eigen::Vector3d accelrate = accel - this->sysNominalState_.accelBias;
    Eigen::Vector3d kv1 = rotmatI2GBeginTime * accelrate + this->sysNominalState_.gravityAtG;
    Eigen::Vector3d kv2 = rotmatI2GHalfTime * accelrate + this->sysNominalState_.gravityAtG;
    Eigen::Vector3d kv3 = kv2;
    Eigen::Vector3d kv4 = this->sysNominalState_.rotmatI2G * accelrate + this->sysNominalState_.gravityAtG;
    Eigen::Vector3d preVelocity = this->sysNominalState_.velocityAtG;
    this->sysNominalState_.velocityAtG = preVelocity + deltaT / 6 * (kv1 + 2 * kv2 + 2 * kv3 + kv4);

    // 位置更新, 四阶龙格库塔算法
    Eigen::Vector3d kp1 = preVelocity;
    Eigen::Vector3d kp2 = preVelocity + kv1 * deltaT / 2;
    Eigen::Vector3d kp3 = preVelocity + kv2 * deltaT / 2;
    Eigen::Vector3d kp4 = preVelocity + kv3 * deltaT / 2;
    this->sysNominalState_.positionAtG = this->sysNominalState_.positionAtG + deltaT / 6 * (kp1 + 2 * kp2 + 2 * kp3 + kp4);
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::UpdateCovariance(double dtime, const Eigen::Vector3d &accel, const Eigen::Vector3d &gyro)
{
    // 时间间隔计算
    double deltaT = dtime;

    // IMU数据
    Eigen::Vector3d angularVel = gyro - this->sysNominalState_.gyroBias;
    Eigen::Vector3d accelrate = accel - this->sysNominalState_.accelBias;

    // 构造误差状态Jocobian矩阵
    this->sysErrorState_.stateJocobianMat = Eigen::Matrix<double, 18, 18>::Identity();
    this->sysErrorState_.stateJocobianMat.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * deltaT;
    this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 6) = -this->sysNominalState_.rotmatI2G * FBUSEKF::SkewSymmetricMatrix(accelrate) * deltaT;
    this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 9) = -this->sysNominalState_.rotmatI2G * deltaT;
    this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * deltaT;
    this->sysErrorState_.stateJocobianMat.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - FBUSEKF::SkewSymmetricMatrix(angularVel) * deltaT;
    this->sysErrorState_.stateJocobianMat.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * deltaT;

    // 主协方差矩阵更新
    this->sysErrorState_.stateCovariance.block<18, 18>(0, 0) = this->sysErrorState_.stateJocobianMat * this->sysErrorState_.stateCovariance.block<18, 18>(0, 0) *
                                                                   this->sysErrorState_.stateJocobianMat.transpose() +
                                                               this->sysErrorState_.noiseCoefficientMat * this->sysErrorState_.noiseCovariance *
                                                                   this->sysErrorState_.noiseCoefficientMat.transpose();


    // 进一步保证协方差矩阵的对称性
    Eigen::MatrixXd stateCovFixed = (this->sysErrorState_.stateCovariance + this->sysErrorState_.stateCovariance.transpose()) / 2.0;
    this->sysErrorState_.stateCovariance = stateCovFixed;
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::ObservationUpdate()
{
    std::unique_lock<std::mutex> lck(this->imgMutex);
    if (this->detectionResult_.size() > 0)
    {
        LOG(INFO) << "Measurement update at time: " << this->sysNominalState_.timeStamp;

        Eigen::Matrix3d R_I_L = this->cameraInfoList_[0].T_SC.block<3, 3>(0, 0);
        Eigen::Quaterniond Q_I_L(R_I_L);
        Eigen::Vector3d P_L_I = this->cameraInfoList_[0].T_SC.block<3, 1>(0, 3);
        Eigen::Vector3d P_I_L = -R_I_L.transpose() * P_L_I;

        Eigen::Matrix<double, 4, 3> L1 = Eigen::Matrix<double, 4, 3>::Zero();
        L1.block<3, 3>(1, 0) = 0.5 * Eigen::Matrix3d::Identity();
        Eigen::Matrix4d L2 = -1 * Eigen::Matrix4d::Identity();
        L2(0, 0) = 1;
        
        int count = 0;
        int min_dist_id = 0;
        double min_dist = 10;
        double preUsedMarkerDistance = 0;
        double preUsedMarkerDistID = 0;
        for (const auto &detectmarker : this->detectionResult_)
        {
            double distance = detectmarker.positionAtCL.norm();
            if(distance < min_dist)
            {
                min_dist = distance;
                min_dist_id = count;
            }
            if(detectmarker.markerID == this->preUsedMarkerID_)
            {
                preUsedMarkerDistance = distance;
                preUsedMarkerDistID = count;
            }
            count++;
        }
        
        if((FBUSEKF::Absolute(preUsedMarkerDistance - min_dist)<this->ekfParam_.fcpMarkerSwitchThres)&&(preUsedMarkerDistance!=0))
        {
            min_dist_id = preUsedMarkerDistID;
            LOG(INFO) << "Go on taking the previous marker to update ...";
        }

        FBUSEKF::DetectionResult nearest_marker = this->detectionResult_[min_dist_id];

        int markerID = nearest_marker.markerID;
        // std::cout << "Update Marker " << markerID << std::endl;
        // 确认系统状态中是否包含该Marker，如果不包含，则不需要量测更新，就直接return
        auto markerIter = this->markerPoseServer_.find(markerID);
        if(markerIter == this->markerPoseServer_.end())
            return;

        this->preUsedMarkerID_ = markerID;

        // 实际量测值
        Eigen::Vector3d yP_L_M = nearest_marker.positionAtCL;
        Eigen::Quaterniond yQ_L_M = nearest_marker.quaternionM2CL;

        // 估计值（根据系统状态）
        Eigen::Vector3d P_G_M = markerIter->second.positionAtG;
        Eigen::Vector3d P_G_I = this->sysNominalState_.positionAtG;
        Eigen::Vector3d hP_L_M = R_I_L * this->sysNominalState_.rotmatI2G.transpose() *
                                (P_G_M - P_G_I - this->sysNominalState_.rotmatI2G * P_I_L);
        Eigen::Quaterniond hQ_L_M = Q_I_L * this->sysNominalState_.quaternionI2G.conjugate() * markerIter->second.quaternionM2G;

        // 量测雅克比矩阵的构造
        size_t col = this->sysErrorState_.stateCovariance.cols();
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(7, col);
        H.block<3, 3>(0, 0) = -R_I_L * this->sysNominalState_.rotmatI2G.transpose();
        H.block<3, 3>(0, 6) = R_I_L * FBUSEKF::SkewSymmetricMatrix(this->sysNominalState_.rotmatI2G.transpose() * (P_G_M - P_G_I));
        H.block<4, 3>(3, 6) = QuaternionRightProductMatrix(markerIter->second.quaternionM2G) *
                            QuaternionLeftProductMatrix(Q_I_L) * L2 * QuaternionLeftProductMatrix(this->sysNominalState_.quaternionI2G) * L1;

        // 这段代码写在这里的原因是，同一个姿态可以对应两个相反的四元数，必须要将他们符号统一，才可以顺利实现更新
        // 这段代码的基本原理就是：判断两个四元数是否同号，同号就不用处理，异号就需要加个负号
        double k1 = (yQ_L_M.w()-hQ_L_M.w())*(yQ_L_M.w()-hQ_L_M.w()) + (yQ_L_M.x()-hQ_L_M.x())*(yQ_L_M.x()-hQ_L_M.x())
                    + (yQ_L_M.y()-hQ_L_M.y())*(yQ_L_M.y()-hQ_L_M.y()) + (yQ_L_M.z()-hQ_L_M.z())*(yQ_L_M.z()-hQ_L_M.z());
        double k2 = (yQ_L_M.w()+hQ_L_M.w())*(yQ_L_M.w()+hQ_L_M.w()) + (yQ_L_M.x()+hQ_L_M.x())*(yQ_L_M.x()+hQ_L_M.x())
                    + (yQ_L_M.y()+hQ_L_M.y())*(yQ_L_M.y()+hQ_L_M.y()) + (yQ_L_M.z()+hQ_L_M.z())*(yQ_L_M.z()+hQ_L_M.z());
        if(k1 > k2) 
        {
            hQ_L_M = Eigen::Quaterniond(-hQ_L_M.w(), -hQ_L_M.x(), -hQ_L_M.y(), -hQ_L_M.z());
            H.block<4, 3>(3, 6) = -H.block<4, 3>(3, 6);
        }

        // 计算Kalman增益
        const Eigen::MatrixXd &P = this->sysErrorState_.stateCovariance;
        Eigen::MatrixXd S = H * P * H.transpose() + this->sysErrorState_.observeNoiseCovariance;
        Eigen::MatrixXd K_transpose = S.ldlt().solve(H * P);
        Eigen::MatrixXd K = K_transpose.transpose(); // 上述代码是换了一种方式求逆
        // Eigen::MatrixXd K = P * H.transpose() * S.inverse();

        // 计算误差状态
        Eigen::Matrix<double, 7, 1> residual;
        residual.block<3, 1>(0, 0) = yP_L_M - hP_L_M;
        residual(3, 0) = yQ_L_M.w() - hQ_L_M.w();
        residual(4, 0) = yQ_L_M.x() - hQ_L_M.x();
        residual(5, 0) = yQ_L_M.y() - hQ_L_M.y();
        residual(6, 0) = yQ_L_M.z() - hQ_L_M.z();

        Eigen::MatrixXd deltaX = K * residual;

        // 修正系统状态
        this->sysNominalState_.positionAtG = this->sysNominalState_.positionAtG + deltaX.block<3, 1>(0, 0);
        this->sysNominalState_.velocityAtG = this->sysNominalState_.velocityAtG + deltaX.block<3, 1>(3, 0);
        Eigen::Vector3d deltaTheta1 = deltaX.block<3, 1>(6, 0);
        this->sysNominalState_.quaternionI2G = this->sysNominalState_.quaternionI2G * VectorToQuaterniond(deltaTheta1);
        this->sysNominalState_.quaternionI2G.normalize();
        this->sysNominalState_.accelBias = this->sysNominalState_.accelBias + deltaX.block<3, 1>(9, 0);
        this->sysNominalState_.gyroBias = this->sysNominalState_.gyroBias + deltaX.block<3, 1>(12, 0);
        this->sysNominalState_.gravityAtG = this->sysNominalState_.gravityAtG + deltaX.block<3, 1>(15, 0);
        // 更新协方差矩阵
        this->sysErrorState_.stateCovariance = (Eigen::MatrixXd::Identity(col, col) - K * H) * this->sysErrorState_.stateCovariance;

        // 进一步保证协方差矩阵的对称性
        Eigen::MatrixXd stateCovFixed = (this->sysErrorState_.stateCovariance + this->sysErrorState_.stateCovariance.transpose()) / 2.0;
        this->sysErrorState_.stateCovariance = stateCovFixed;
                 
    }

    LOG(INFO) << "    IMU Position: " << this->sysNominalState_.positionAtG[0] << ", " << this->sysNominalState_.positionAtG[1] << ", "
              << this->sysNominalState_.positionAtG[2];
    LOG(INFO) << "  IMU Quaternion: " << this->sysNominalState_.quaternionI2G.w() << ", " << this->sysNominalState_.quaternionI2G.x() << ", "
              << this->sysNominalState_.quaternionI2G.y() << ", " << this->sysNominalState_.quaternionI2G.z();
    LOG(INFO) << "IMU RotateMatrix: " << this->sysNominalState_.rotmatI2G;
    LOG(INFO) << "      Accel Bias: " << this->sysNominalState_.accelBias[0] << ", " << this->sysNominalState_.accelBias[1] << ", "
              << this->sysNominalState_.accelBias[2];
    LOG(INFO) << "       Gyro Bias: " << this->sysNominalState_.gyroBias[0] << ", " << this->sysNominalState_.gyroBias[1] << ", "
              << this->sysNominalState_.gyroBias[2];
    LOG(INFO) << "         Gravity: " << this->sysNominalState_.gravityAtG[0] << ", " << this->sysNominalState_.gravityAtG[1] << ", "
              << this->sysNominalState_.gravityAtG[2];
}
/****
 *
 *
 *
 ****/
void FBUSEKF::FILTER::ClearImuBuffer(std::vector<FBUSEKF::IMUData>::iterator begin, std::vector<FBUSEKF::IMUData>::iterator end)
{
    // std::unique_lock<std::mutex> lck(this->imuMutex); // 不能加两层的锁
    this->imuMeasuementBuffer_.erase(begin, end);
}
