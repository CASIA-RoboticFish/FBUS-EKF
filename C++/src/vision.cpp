#include "vision.hpp"

void FBUSEKF::VISION::SetImageData(FBUSEKF::ImageData imgdata)
{
    std::unique_lock<std::mutex> lck(this->imageMutex);
    this->imageMeasuementQueue_.push(imgdata);
}
/****
 *
 *
 *
 ****/
cv::Mat FBUSEKF::VISION::GetImageData()
{
    std::unique_lock<std::mutex> lck(this->imageMutex);
    // 这里存在bug,以后要修改
    if (this->imageMeasuementQueue_.size() > 0)
        return this->imageMeasuementQueue_.front().image0;
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::StartVisionThread()
{
    this->visionThread_ = std::thread(std::bind(&FBUSEKF::VISION::VisionThreadFunction, this));
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::VisionThreadFunction()
{
    LOG(INFO) << "Vision thread has been started ~~~" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待1000ms, 收集IMU数据, 进行初始化
    while (1)
    {
        std::unique_lock<std::mutex> lck(this->imageMutex);
        this->imageCondVar.wait(lck);

        if (this->imageMeasuementQueue_.size() > IMAGE_QUEUE_MAX_SIZE)
        {
            while (this->imageMeasuementQueue_.size() >= IMAGE_QUEUE_MAX_SIZE)
            {
                this->imageMeasuementQueue_.pop();
            }
        }

        LOG(INFO) << "Carryout vision algorithm at " << std::fixed << std::setprecision(6) << this->imageMeasuementQueue_.front().timeStamp << std::endl;

        // 清除检测结果
        this->ClearMarkerMap();
        // LOG(INFO) << "Marker num (before detect): " << this->markerObservation_.size();
        // 检测Marker
        this->DetectArucoTag();

        // 三角化Marker角点
        if(this->isUnderwaterMode_)
        {      
            this->RefractionTriangulation();
        }
        else
        {
            this->NormalTriangulation();
        }

        // 计算Marker的位置和旋转
        this->ComputeMarkerPose();

        //
        this->RejectInvalidMarker();

        // 显示图像
        this->DisplayImage();

        

        // 弹出已检测的图像
        this->preImageTime_ = this->imageMeasuementQueue_.front().timeStamp;
        this->imageMeasuementQueue_.pop();


        // 输出检测结果
        // LOG(INFO) << "Marker num (after detect): " << this->markerObservation_.size();
        FBUSEKF::DetectionResultList resultList;
        for (auto iter = this->markerObservation_.begin(); iter != this->markerObservation_.end(); iter++)
        {
            if (iter->second.isComputePose)
            {
                FBUSEKF::DetectionResult result;
                result.timeStamp = iter->second.timeStamp;
                result.markerID = iter->first;
                result.markerSize = iter->second.markerSize;
                result.positionAtCL = iter->second.positionAtCL;
                result.quaternionM2CL = iter->second.quaternionM2CL;
                result.rotmatM2L = iter->second.rotmatM2L;
                resultList.push_back(result);

#ifdef OPEN_DATA_RECORDING
                std::ofstream ofs1;
                ofs1.open("data/image.txt", std::ios::app);
                ofs1 << result.timeStamp << " " << result.markerID << " " << result.positionAtCL[0] << " " << result.positionAtCL[1]
                    << " " << result.positionAtCL[2] << " " << result.quaternionM2CL.w()<< " " << result.quaternionM2CL.x()
                    << " " << result.quaternionM2CL.y() << " " << result.quaternionM2CL.z() << std::endl;
                ofs1.close();

                std::ofstream ofs2;
                ofs2.open("data/corners.txt", std::ios::app);
                ofs2 << result.timeStamp << " " << result.markerID << " " 
                   << iter->second.cornerObservationLeft[0].x << " " << iter->second.cornerObservationLeft[0].y << " " 
                   << iter->second.cornerObservationLeft[1].x << " " << iter->second.cornerObservationLeft[1].y << " " 
                   << iter->second.cornerObservationLeft[2].x << " " << iter->second.cornerObservationLeft[2].y << " " 
                   << iter->second.cornerObservationLeft[3].x << " " << iter->second.cornerObservationLeft[3].y << " " 
                   << iter->second.cornerObservationRight[0].x << " " << iter->second.cornerObservationRight[0].y << " "
                   << iter->second.cornerObservationRight[1].x << " " << iter->second.cornerObservationRight[1].y << " " 
                   << iter->second.cornerObservationRight[2].x << " " << iter->second.cornerObservationRight[2].y << " " 
                   << iter->second.cornerObservationRight[3].x << " " << iter->second.cornerObservationRight[3].y << std::endl;
                // ofs2 << result.timeStamp << " " << result.markerID << " " 
                //      << iter->second.cornerPositionAtCL[0][0] << " " << iter->second.cornerPositionAtCL[0][1] << " " << iter->second.cornerPositionAtCL[0][2] << " "
                //      << iter->second.cornerPositionAtCL[1][0] << " " << iter->second.cornerPositionAtCL[1][1] << " " << iter->second.cornerPositionAtCL[1][2] << " "
                //      << iter->second.cornerPositionAtCL[2][0] << " " << iter->second.cornerPositionAtCL[2][1] << " " << iter->second.cornerPositionAtCL[2][2] << " "
                //      << iter->second.cornerPositionAtCL[3][0] << " " << iter->second.cornerPositionAtCL[3][1] << " " << iter->second.cornerPositionAtCL[3][2] << std::endl;

                ofs2.close();

#endif

                LOG(INFO) << "Detect Marker ID: " << iter->second.markerID;
                LOG(INFO) << ", Position: [" << iter->second.positionAtCL[0] << ", " << iter->second.positionAtCL[1]
                          << ", " << iter->second.positionAtCL[2] << "], Quaternion: x=" << iter->second.quaternionM2CL.x() << ", y="
                          << iter->second.quaternionM2CL.y() << ", z=" << iter->second.quaternionM2CL.z() << ", w=" << iter->second.quaternionM2CL.w();
            }
        }
        if (resultList.size() != 0)
        {
            this->pFilter->SetDetectionResult(resultList);
            this->pFilter->SetDetectionResultUpdated();
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::DetectArucoTag()
{
    if (this->imageMeasuementQueue_.size() > 0)
    {

        cv::Mat leftImg = this->imageMeasuementQueue_.front().image0;
        cv::Mat rightImg = this->imageMeasuementQueue_.front().image1;

#ifdef OPEN_IMAGE_RECORDING
        cv::imwrite("data/image/left_" + std::to_string(this->imageMeasuementQueue_.front().timeStamp) + ".jpg", leftImg);
        cv::imwrite("data/image/right_" + std::to_string(this->imageMeasuementQueue_.front().timeStamp) + ".jpg", rightImg);
#endif

        // 左目图像处理
        cv::Mat cameraMat0;
        cv::eigen2cv(this->cameraInfo_[0].K, cameraMat0);
        cv::Mat distortion0;
        cv::eigen2cv(this->cameraInfo_[0].D, distortion0);
        cv::Size imageSize(640, 400);
        aruco::CameraParameters cameraParam0;
        cameraParam0.setParams(cameraMat0, distortion0, imageSize);

        if (this->isTrackerApplied_) // 是否启用Marker跟踪模式
        {            
            std::map<int, cv::Ptr<TrackerImpl>> trackers0 = this->markerTracker0_.trackFBUSEKF(leftImg, 0.0f, this->isReDetect_);
            for (auto t : trackers0)
            {
                aruco::Marker m = t.second->getMarker();

                // 检查这个Marker是不是在我们所选用的Marker范围内，即检测到的这个Marker是不是有效的Marker
                std::vector<FBUSEKF::MarkerID>::iterator iterID = std::find(this->effectiveMarkerList_.begin(), this->effectiveMarkerList_.end(), m.id);
                if (iterID == this->effectiveMarkerList_.end())
                    continue;

                std::vector<cv::Point2f> distortedCorners;
                bool isCornersInForbiddenField = false;
                for (int i = 0; i < 4; i++)
                {
                    if(m[i].x < this->visionParam_.edgeForbiddenThres || m[i].x > (this->visionParam_.frameWidth - this->visionParam_.edgeForbiddenThres) ||
                       m[i].y < this->visionParam_.edgeForbiddenThres || m[i].y > (this->visionParam_.frameHeight - this->visionParam_.edgeForbiddenThres))
                    {
                        isCornersInForbiddenField = true;
                        break;
                    }
                    distortedCorners.push_back(cv::Point2f(m[i].x, m[i].y));
                }
                if(isCornersInForbiddenField)
                    continue;

                FBUSEKF::Marker marker;
                marker.timeStamp = this->imageMeasuementQueue_.front().timeStamp;
                marker.markerID = m.id;
                marker.isDetectedLeft = true;
                marker.markerSize = this->markerSize_;
                cv::cornerSubPix(leftImg, distortedCorners, cv::Size(5,5), cv::Size(-1, -1),cv::TermCriteria( cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
                cv::fisheye::undistortPoints(distortedCorners, marker.cornerObservationLeft, cameraMat0, distortion0);
                this->markerObservation_.insert(std::pair<MarkerID, Marker>(m.id, marker));

                if (this->showImage_)
                {
                    m.draw(leftImg);
                    auto iter = this->markerObservation_.find(m.id);
                    for (int i = 0; i < 4; i++)
                    {
                        cv::Point2d pt;
                        pt.x = iter->second.cornerObservationLeft[i].x * cameraMat0.at<double>(0, 0) + cameraMat0.at<double>(0, 2);
                        pt.y = iter->second.cornerObservationLeft[i].y * cameraMat0.at<double>(1, 1) + cameraMat0.at<double>(1, 2);
                        cv::circle(leftImg, pt, 5, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
                        // cv::Rect r(distortedCorners[i].x, distortedCorners[i].y, 10, 10);
                        // cv::rectangle(leftImg, r, cv::Scalar(255, 0, 0), -1);
                    }
                }
            }
        }
        else
        {
            auto markers0 = this->markerDetector0_.detect(leftImg, cameraParam0, this->markerSize_);
            for (auto m : markers0)
            {
                // 检查这个Marker是不是在我们所选用的Marker范围内，即检测到的这个Marker是不是有效的Marker
                std::vector<FBUSEKF::MarkerID>::iterator iterID = std::find(this->effectiveMarkerList_.begin(), this->effectiveMarkerList_.end(), m.id);
                if (iterID == this->effectiveMarkerList_.end())
                    continue;

                std::vector<cv::Point2f> distortedCorners;
                bool isCornersInForbiddenField = false;
                for (int i = 0; i < 4; i++)
                {
                    if(m[i].x < this->visionParam_.edgeForbiddenThres || m[i].x > (this->visionParam_.frameWidth - this->visionParam_.edgeForbiddenThres) ||
                       m[i].y < this->visionParam_.edgeForbiddenThres || m[i].y > (this->visionParam_.frameHeight - this->visionParam_.edgeForbiddenThres))
                    {
                        isCornersInForbiddenField = true;
                        break;
                    }
                    distortedCorners.push_back(cv::Point2f(m[i].x, m[i].y));
                }
                if(isCornersInForbiddenField)
                    continue;

                FBUSEKF::Marker marker;
                marker.timeStamp = this->imageMeasuementQueue_.front().timeStamp;
                marker.markerID = m.id;
                marker.isDetectedLeft = true;
                marker.markerSize = this->markerSize_;
                
                cv::fisheye::undistortPoints(distortedCorners, marker.cornerObservationLeft, cameraMat0, distortion0);
                this->markerObservation_.insert(std::pair<MarkerID, Marker>(m.id, marker));

                if (this->showImage_)
                {
                    m.draw(leftImg);
                    auto iter = this->markerObservation_.find(m.id);
                    for (int i = 0; i < 4; i++)
                    {
                        cv::Point2d pt;
                        pt.x = iter->second.cornerObservationLeft[i].x * cameraMat0.at<double>(0, 0) + cameraMat0.at<double>(0, 2);
                        pt.y = iter->second.cornerObservationLeft[i].y * cameraMat0.at<double>(1, 1) + cameraMat0.at<double>(1, 2);
                        cv::circle(leftImg, pt, 5, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
                        // cv::Rect r(distortedCorners[i].x, distortedCorners[i].y, 10, 10);
                        // cv::rectangle(leftImg, r, cv::Scalar(255, 0, 0), -1);
                    }
                }
            }
        }

        // 右目图像处理
        cv::Mat cameraMat1;
        cv::eigen2cv(this->cameraInfo_[1].K, cameraMat1);
        cv::Mat distortion1;
        cv::eigen2cv(this->cameraInfo_[1].D, distortion1);
        aruco::CameraParameters cameraParam1;
        cameraParam1.setParams(cameraMat1, distortion1, imageSize);

        if (this->isTrackerApplied_)
        {
            std::map<int, cv::Ptr<TrackerImpl>> trackers1 = this->markerTracker1_.trackFBUSEKF(rightImg, 0.0f, this->isReDetect_);
            this->isReDetect_ = false;

            for (auto t : trackers1)
            {
                aruco::Marker m = t.second->getMarker();

                // 检查这个Marker是不是在我们所选用的Marker范围内，即检测到的这个Marker是不是有效的Marker
                std::vector<FBUSEKF::MarkerID>::iterator iterID = std::find(this->effectiveMarkerList_.begin(), this->effectiveMarkerList_.end(), m.id);
                if (iterID == this->effectiveMarkerList_.end())
                    continue;

                auto iter = this->markerObservation_.find(m.id);
                if (iter != this->markerObservation_.end())
                {
                    std::vector<cv::Point2f> distortedCorners;
                    bool isCornersInForbiddenField = false;
                    for (int i = 0; i < 4; i++)
                    {
                        if(m[i].x < this->visionParam_.edgeForbiddenThres || m[i].x > (this->visionParam_.frameWidth - this->visionParam_.edgeForbiddenThres) ||
                        m[i].y < this->visionParam_.edgeForbiddenThres || m[i].y > (this->visionParam_.frameHeight - this->visionParam_.edgeForbiddenThres))
                        {
                            isCornersInForbiddenField = true;
                            break;
                        }
                        distortedCorners.push_back(cv::Point2f(m[i].x, m[i].y));
                    }
                    if(isCornersInForbiddenField)
                        continue;

                    iter->second.timeStamp = this->imageMeasuementQueue_.front().timeStamp;
                    iter->second.markerID = m.id;
                    iter->second.isDetectedRight = true;
                    
                    cv::cornerSubPix(rightImg, distortedCorners, cv::Size(5,5), cv::Size(-1, -1),cv::TermCriteria( cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
                    cv::fisheye::undistortPoints(distortedCorners, iter->second.cornerObservationRight, cameraMat1, distortion1);

                    if (this->showImage_)
                    {
                        m.draw(rightImg);
                        auto iter = this->markerObservation_.find(m.id);
                        for (int i = 0; i < 4; i++)
                        {
                            cv::Point2d pt;
                            pt.x = iter->second.cornerObservationRight[i].x * cameraMat1.at<double>(0, 0) + cameraMat1.at<double>(0, 2);
                            pt.y = iter->second.cornerObservationRight[i].y * cameraMat1.at<double>(1, 1) + cameraMat1.at<double>(1, 2);
                            cv::circle(rightImg, pt, 5, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
                            // cv::Rect r(distortedCorners[i].x, distortedCorners[i].y, 10, 10);
                            // cv::rectangle(rightImg, r, cv::Scalar(255, 0, 0), -1);
                        }
                    }
                }
            }
        }
        else
        {
            auto markers1 = this->markerDetector1_.detect(rightImg, cameraParam1, this->markerSize_);
            for (auto m : markers1)
            {
                // 检查这个Marker是不是在我们所选用的Marker范围内，即检测到的这个Marker是不是有效的Marker
                std::vector<FBUSEKF::MarkerID>::iterator iterID = std::find(this->effectiveMarkerList_.begin(), this->effectiveMarkerList_.end(), m.id);
                if (iterID == this->effectiveMarkerList_.end())
                    continue;

                auto iter = this->markerObservation_.find(m.id);
                if (iter != this->markerObservation_.end())
                {
                    std::vector<cv::Point2f> distortedCorners;
                    bool isCornersInForbiddenField = false;
                    for (int i = 0; i < 4; i++)
                    {
                        if(m[i].x < this->visionParam_.edgeForbiddenThres || m[i].x > (this->visionParam_.frameWidth - this->visionParam_.edgeForbiddenThres) ||
                        m[i].y < this->visionParam_.edgeForbiddenThres || m[i].y > (this->visionParam_.frameHeight - this->visionParam_.edgeForbiddenThres))
                        {
                            isCornersInForbiddenField = true;
                            break;
                        }
                        distortedCorners.push_back(cv::Point2f(m[i].x, m[i].y));
                    }
                    if(isCornersInForbiddenField)
                        continue;

                    iter->second.timeStamp = this->imageMeasuementQueue_.front().timeStamp;
                    iter->second.markerID = m.id;
                    iter->second.isDetectedRight = true;
                    
                    cv::fisheye::undistortPoints(distortedCorners, iter->second.cornerObservationRight, cameraMat1, distortion1);

                    if (this->showImage_)
                    {
                        m.draw(rightImg);
                        auto iter = this->markerObservation_.find(m.id);
                        for (int i = 0; i < 4; i++)
                        {
                            cv::Point2d pt;
                            pt.x = iter->second.cornerObservationRight[i].x * cameraMat1.at<double>(0, 0) + cameraMat1.at<double>(0, 2);
                            pt.y = iter->second.cornerObservationRight[i].y * cameraMat1.at<double>(1, 1) + cameraMat1.at<double>(1, 2);
                            cv::circle(rightImg, pt, 5, cv::Scalar(255, 0, 0), -1); //第五个参数我设为-1，表明这是个实点。
                            // cv::Rect r(distortedCorners[i].x, distortedCorners[i].y, 10, 10);
                            // cv::rectangle(rightImg, r, cv::Scalar(255, 0, 0), -1);
                        }
                    }
                }
            }
        }
    }
}
/****T_L_R
 *
 *
 *
 ****/
void FBUSEKF::VISION::NormalTriangulation()
{
    if (this->markerObservation_.size() != 0)
    {
        Eigen::Matrix<double, 3, 4> T0 = Eigen::Matrix<double, 3, 4>::Zero();
        T0.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d T_I_L = this->cameraInfo_[0].T_SC.block<3, 3>(0, 0);
        Eigen::Vector3d P_L_I = this->cameraInfo_[0].T_SC.block<3, 1>(0, 3);
        Eigen::Matrix3d T_I_R = this->cameraInfo_[1].T_SC.block<3, 3>(0, 0);
        Eigen::Vector3d P_R_I = this->cameraInfo_[1].T_SC.block<3, 1>(0, 3);
        Eigen::Matrix<double, 3, 4> T_L_R = Eigen::Matrix<double, 3, 4>::Zero();
        T_L_R.block<3, 3>(0, 0) = T_I_R * T_I_L.transpose();
        T_L_R.block<3, 1>(0, 3) = P_L_I - T_L_R.block<3, 3>(0, 0) * P_R_I;
        for (auto iter = this->markerObservation_.begin(); iter != this->markerObservation_.end(); iter++)
        {
            MarkerID markerID = iter->first;
            bool isOutDectionRange = false;
            if (iter->second.isDetectedLeft && iter->second.isDetectedRight) // 如果左右目都检测到了该Marker，开始执行角点三角化
            {
                for (int i = 0; i < 4; i++)
                {
                    // Eigen::Matrix3d R_I_C; // Image frame to Camera frame
                    // R_I_C << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
                    // R_I_C << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
                    Eigen::Vector3d leftPointatI(iter->second.cornerObservationLeft[i].x, iter->second.cornerObservationLeft[i].y, 1);
                    // Eigen::Vector3d leftPointatC = R_I_C * leftPointatI;
                    Eigen::Matrix3d leftPointSkewMat = FBUSEKF::SkewSymmetricMatrix(leftPointatI);
                    Eigen::Vector3d rightPointatI(iter->second.cornerObservationRight[i].x, iter->second.cornerObservationRight[i].y, 1);
                    // Eigen::Vector3d rightPointatC = R_I_C * rightPointatI;
                    Eigen::Matrix3d rightPointSkewMat = FBUSEKF::SkewSymmetricMatrix(rightPointatI);

                    Eigen::Matrix<double, 6, 4> A;
                    A.block<3, 4>(0, 0) = leftPointSkewMat * T0;
                    A.block<3, 4>(3, 0) = rightPointSkewMat * T_L_R;

                    // SVD
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
                    Eigen::Matrix4d V = svd.matrixV();
                    Eigen::MatrixXd U = svd.matrixU();
                    //Eigen::MatrixXd S = U.inverse() * A * V.transpose().inverse();

                    Eigen::Vector4d P = V.col(3);

                    if (P[3] == 0)
                    {
                        continue;
                    }

                    Eigen::Vector3d Pn = P.block<3, 1>(0, 0) / P[3];
                    Eigen::Matrix3d R_I_C; // Image frame to Camera frame
                    R_I_C << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
                    Pn = R_I_C * Pn;
                    iter->second.cornerPositionAtCL[i] = Signum(Pn[2]) * Pn;

                    double distance = Pn.norm();
                    if(distance > this->visionParam_.makrerDectDistThres)
                    {
                        isOutDectionRange = true;
                        break;
                    }
                }
                // Important, Reset flag
                if(!isOutDectionRange)
                    iter->second.isTriangulation = true;
            }
            // Important, Reset flag
            iter->second.isDetectedLeft = false;
            iter->second.isDetectedRight = false;
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::RefractionTriangulation()
{
    if (this->markerObservation_.size() != 0)
    {
        Eigen::Matrix3d R_I_L = this->cameraInfo_[0].T_SC.block<3, 3>(0, 0);
        Eigen::Vector3d P_L_I = this->cameraInfo_[0].T_SC.block<3, 1>(0, 3);
        Eigen::Matrix3d R_I_R = this->cameraInfo_[1].T_SC.block<3, 3>(0, 0);
        Eigen::Vector3d P_R_I = this->cameraInfo_[1].T_SC.block<3, 1>(0, 3);
        Eigen::Matrix3d R_R_L = R_I_L * R_I_R.transpose();
        Eigen::Vector3d P_L_R = P_L_I - R_R_L * P_R_I;
        for (auto iter = this->markerObservation_.begin(); iter != this->markerObservation_.end(); iter++)
        {
            MarkerID markerID = iter->first;
            bool isOutDectionRange = false;
            if (iter->second.isDetectedLeft && iter->second.isDetectedRight) // 如果左右目都检测到了该Marker，开始执行角点三角化
            {
                for (int i = 0; i < 4; i++)
                {
                    // Eigen::Matrix3d R_I_C; // Image frame to Camera frame
                    // R_I_C << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
                    Eigen::Vector3d leftPoint(iter->second.cornerObservationLeft[i].x, iter->second.cornerObservationLeft[i].y, 1);
                    // leftPoint = R_I_C * leftPoint;
                    Eigen::Vector3d rightPoint(iter->second.cornerObservationRight[i].x, iter->second.cornerObservationRight[i].y, 1);
                    // rightPoint = R_I_C * rightPoint;

                    // compute r0
                    Eigen::Vector3d r0_L = leftPoint / leftPoint.norm();
                    Eigen::Vector3d r0_R = rightPoint / rightPoint.norm();

                    // normal vector
                    Eigen::Vector3d normalVec = this->refractInfo_.normalVec;

                    // compute r1
                    double alpha0 = this->refractInfo_.airRefRatio / this->refractInfo_.glassRefRatio;
                    double v0_L = r0_L.transpose() * normalVec;
                    double beta0_L = 0;
                    double v0_R = r0_R.transpose() * normalVec;
                    double beta0_R = 0;

                    if(this->refractInfo_.airRefRatio < this->refractInfo_.glassRefRatio)
                    {
                        beta0_L = sqrt(1 - alpha0 * alpha0 * (1 - v0_L * v0_L)) - alpha0 * v0_L;
                        beta0_R = sqrt(1 - alpha0 * alpha0 * (1 - v0_R * v0_R)) - alpha0 * v0_R;
                    }
                    else
                    {
                        beta0_L = alpha0 * v0_L - sqrt(1 - alpha0 * alpha0 * (1 - v0_L * v0_L));
                        beta0_R = alpha0 * v0_R - sqrt(1 - alpha0 * alpha0 * (1 - v0_R * v0_R));
                    }

                    Eigen::Vector3d r1_L = alpha0 * r0_L + beta0_L * normalVec;
                    Eigen::Vector3d r1_R = alpha0 * r0_R + beta0_R * normalVec;

                    // compute r2
                    double alpha1 = this->refractInfo_.glassRefRatio / this->refractInfo_.waterRefRatio;
                    double v1_L = r1_L.transpose() * normalVec;
                    double beta1_L = 0;
                    double v1_R = r1_R.transpose() * normalVec;
                    double beta1_R = 0;
                    if(this->refractInfo_.glassRefRatio > this->refractInfo_.waterRefRatio)
                    {
                        beta1_L = sqrt(1 - alpha1 * alpha1 * (1 - v1_L * v1_L)) - alpha1 * v1_L;
                        beta1_R = sqrt(1 - alpha1 * alpha1 * (1 - v1_R * v1_R)) - alpha1 * v1_R;
                    }
                    else
                    {
                        beta1_L = alpha1 * v1_L - sqrt(1 - alpha1 * alpha1 * (1 - v1_L * v1_L));
                        beta1_R = alpha1 * v1_R - sqrt(1 - alpha1 * alpha1 * (1 - v1_R * v1_R));
                    }
                    
                    Eigen::Vector3d r2_L = alpha1 * r1_L + beta1_L * normalVec;
                    Eigen::Vector3d r2_R = alpha1 * r1_R + beta1_R * normalVec;

                    // comupte P
                    double d0 = this->refractInfo_.airThickness;
                    double d1 = this->refractInfo_.glassThickness;
                    Eigen::Vector3d P0_L = (d0 / v0_L) * r0_L;
                    Eigen::Vector3d P0_R = (d0 / v0_R) * r0_R;

                    Eigen::Vector3d P1_L = P0_L + (d1 / v1_L) * r1_L;
                    Eigen::Vector3d P1_R = P0_R + (d1 / v1_R) * r1_R;

                    // transform
                    Eigen::Vector3d r2_R_on_L = R_R_L * r2_R;
                    Eigen::Vector3d P1_R_on_L = P_L_R + R_R_L * P1_R;

                    //
                    Eigen::Matrix3d X1 = Eigen::Matrix3d::Zero();
                    X1(0,0) = r2_L(1)*r2_R_on_L(2) - r2_L(2) * r2_R_on_L(1);
                    X1(0,1) = P1_R_on_L(0) - P1_L(0);
                    X1(0,2) = r2_R_on_L(0);
                    X1(1,0) = r2_L(2)*r2_R_on_L(0) - r2_L(0) * r2_R_on_L(2);
                    X1(1,1) = P1_R_on_L(1) - P1_L(1);
                    X1(1,2) = r2_R_on_L(1);
                    X1(2,0) = r2_L(0)*r2_R_on_L(1) - r2_L(1) * r2_R_on_L(0);
                    X1(2,1) = P1_R_on_L(2) - P1_L(2);
                    X1(2,2) = r2_R_on_L(2);

                    Eigen::Matrix3d X2 = Eigen::Matrix3d::Zero();
                    X2(0,0) = r2_L(1)*r2_R_on_L(2) - r2_L(2) * r2_R_on_L(1);
                    X2(0,2) = P1_R_on_L(0) - P1_L(0);
                    X2(0,1) = r2_L(0);
                    X2(1,0) = r2_L(2)*r2_R_on_L(0) - r2_L(0) * r2_R_on_L(2);
                    X2(1,2) = P1_R_on_L(1) - P1_L(1);
                    X2(1,1) = r2_L(1);
                    X2(2,0) = r2_L(0)*r2_R_on_L(1) - r2_L(1) * r2_R_on_L(0);
                    X2(2,2) = P1_R_on_L(2) - P1_L(2);
                    X2(2,1) = r2_L(2);

                    Eigen::Matrix3d X3 = Eigen::Matrix3d::Zero();
                    X3(0,0) = r2_L(1)*r2_R_on_L(2) - r2_L(2) * r2_R_on_L(1);
                    X3(0,2) = r2_R_on_L(0);
                    X3(0,1) = r2_L(0);
                    X3(1,0) = r2_L(2)*r2_R_on_L(0) - r2_L(0) * r2_R_on_L(2);
                    X3(1,2) = r2_R_on_L(1);
                    X3(1,1) = r2_L(1);
                    X3(2,0) = r2_L(0)*r2_R_on_L(1) - r2_L(1) * r2_R_on_L(0);
                    X3(2,2) = r2_R_on_L(2);
                    X3(2,1) = r2_L(2);

                    double t1 = X1.determinant() / X3.determinant();
                    double t2 = -X2.determinant() / X3.determinant();

                    Eigen::Vector3d P = 0.5*(P1_L+t1*r2_L+P1_R_on_L+t2*r2_R_on_L);
                    
                    Eigen::Matrix3d R_I_C; // Image frame to Camera frame
                    R_I_C << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
                    iter->second.cornerPositionAtCL[i] =  R_I_C * P;

                    double distance = P.norm();
                    if(distance > this->visionParam_.makrerDectDistThres)
                    {
                        isOutDectionRange = true;
                        break;
                    }
          
                }
                // Important, Reset flag
                if(!isOutDectionRange)
                    iter->second.isTriangulation = true;
            }
            // Important, Reset flag
            iter->second.isDetectedLeft = false;
            iter->second.isDetectedRight = false;
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::ComputeMarkerPose()
{
    if (this->markerObservation_.size() != 0)
    {
        for (auto iter = this->markerObservation_.begin(); iter != this->markerObservation_.end(); iter++)
        {
            MarkerID markerID = iter->first;
            if (iter->second.isTriangulation) // 如果角点三角化成功，开始计算Marker的位置和姿态
            {

                // 第一步，计算Z轴矢量
                Eigen::Vector3d v1 = iter->second.cornerPositionAtCL[1] - iter->second.cornerPositionAtCL[0];
                Eigen::Vector3d v2 = iter->second.cornerPositionAtCL[2] - iter->second.cornerPositionAtCL[0];
                Eigen::Vector3d v3 = iter->second.cornerPositionAtCL[3] - iter->second.cornerPositionAtCL[0];
                Eigen::Vector3d v4 = iter->second.cornerPositionAtCL[2] - iter->second.cornerPositionAtCL[1];
                Eigen::Vector3d v5 = iter->second.cornerPositionAtCL[3] - iter->second.cornerPositionAtCL[1];
                Eigen::Vector3d v6 = iter->second.cornerPositionAtCL[3] - iter->second.cornerPositionAtCL[2];

                // Threshold judgement 1
                // // pai chu yi xie unprecision marker
                // double d1 = v1.norm();
                // double d2 = v4.norm();
                // double d3 = v6.norm();
                // double d4 = v3.norm();
                // if( Absolute(d1-this->visionParam_.markerSize) > this->visionParam_.markerSizeThres || Absolute(d2-this->visionParam_.markerSize) > this->visionParam_.markerSizeThres || 
                //     Absolute(d3-this->visionParam_.markerSize) > this->visionParam_.markerSizeThres || Absolute(d4-this->visionParam_.markerSize) > this->visionParam_.markerSizeThres)
                // {
                //     iter->second.isTriangulation = false;
                //     iter->second.isComputePose = false;
                //     this->isReDetect_ = true;
                //     LOG(INFO) << "Reset marker tracker is carried out, Error code: 1";
                //     continue; 
                // }
                
                // Threshold judgement 2
                // double d12 = FBUSEKF::Absolute(d2 - d1);
                // double d13 = FBUSEKF::Absolute(d3 - d1);
                // double d14 = FBUSEKF::Absolute(d4 - d1);
                // double d23 = FBUSEKF::Absolute(d3 - d2);
                // double d24 = FBUSEKF::Absolute(d4 - d2);
                // double d34 = FBUSEKF::Absolute(d4 - d3);
                // double d_threshold = this->visionParam_.markerSizeDiffThres;
                // if( d12 > d_threshold || d13 > d_threshold || d14 > d_threshold || d23 > d_threshold || d24 > d_threshold || d34 > d_threshold)
                // {
                //     iter->second.isTriangulation = false;
                //     iter->second.isComputePose = false;
                //     this->isReDetect_ = true;
                //     LOG(INFO) << "Reset marker tracker is carried out, Error code: 2";
                //     continue;
                // }

                

                Eigen::Matrix3d M = v1 * v1.transpose() + v2 * v2.transpose() + v3 * v3.transpose() + v4 * v4.transpose() + v5 * v5.transpose() + v6 * v6.transpose();

                Eigen::EigenSolver<Eigen::Matrix3d> solver(M);
                int colIndex = 0;
                if (solver.eigenvalues()[0].real() < solver.eigenvalues()[1].real())
                {
                    if (solver.eigenvalues()[0].real() < solver.eigenvalues()[2].real())
                        colIndex = 0;
                    else
                        colIndex = 2;
                }
                else
                {
                    if (solver.eigenvalues()[1].real() < solver.eigenvalues()[2].real())
                        colIndex = 1;
                    else
                        colIndex = 2;
                }

                Eigen::Vector3d Zaxis = solver.eigenvectors().col(colIndex).real();
                if (Zaxis[2] > 0.1)
                {
                    Zaxis = -1 * Zaxis;
                }
                else if(Zaxis[2] < -0.1)
                {
                    Zaxis = Zaxis;
                }
                else
                {
                    Zaxis = -FBUSEKF::Signum(iter->second.cornerPositionAtCL[0][0]) * FBUSEKF::Signum(Zaxis[0])*Zaxis;
                    LOG(INFO) << "Normal vector is determined by the Y element";
                }
                // 第二步，计算Marker平面的最优参数D
                double D = 0.25 * Zaxis.transpose() * (iter->second.cornerPositionAtCL[0] + iter->second.cornerPositionAtCL[1] + iter->second.cornerPositionAtCL[2] + iter->second.cornerPositionAtCL[3]);

                // 第三步，计算四个角点在Marker平面上的投影
                double t1 = Zaxis.transpose() * iter->second.cornerPositionAtCL[0] - D;
                Eigen::Vector3d P1 = iter->second.cornerPositionAtCL[0] - t1 * Zaxis;
                double t2 = Zaxis.transpose() * iter->second.cornerPositionAtCL[1] - D;
                Eigen::Vector3d P2 = iter->second.cornerPositionAtCL[1] - t2 * Zaxis;
                double t3 = Zaxis.transpose() * iter->second.cornerPositionAtCL[2] - D;
                Eigen::Vector3d P3 = iter->second.cornerPositionAtCL[2] - t3 * Zaxis;
                double t4 = Zaxis.transpose() * iter->second.cornerPositionAtCL[3] - D;
                Eigen::Vector3d P4 = iter->second.cornerPositionAtCL[3] - t4 * Zaxis;

                // Threshold judgement 3
                // double distanceThreshold = this->visionParam_.cornersProjDistThres;
                // if (t1 > distanceThreshold || t1 < -distanceThreshold || t2 > distanceThreshold || t2 < -distanceThreshold ||
                //     t3 > distanceThreshold || t3 < -distanceThreshold || t4 > distanceThreshold || t4 < -distanceThreshold)
                // {
                //     iter->second.isTriangulation = false;
                //     iter->second.isComputePose = false;
                //     this->isReDetect_ = true;
                //     LOG(INFO) << "Reset marker tracker is carried out, Error code: 3";
                //     continue;
                // }

                // 第四步，计算X轴矢量和Y轴矢量
                Eigen::Vector3d V12 = P2 - P1;
                Eigen::Vector3d V14 = P4 - P1;
                Eigen::Vector3d m = V12 / V12.norm() + V14 / V14.norm();

                Eigen::AngleAxisd angleaxis(-M_PI / 4, Zaxis);
                Eigen::Matrix3d Rm = angleaxis.matrix();

                Eigen::Vector3d Xaxis = Rm * m / m.norm();  // X
                Eigen::Vector3d Yaxis = Zaxis.cross(Xaxis); // Y

                // 第五步，计算Marker的位置，旋转矩阵，姿态四元数
                Eigen::Matrix3d R_M_L;
                R_M_L.block<3, 1>(0, 0) = Xaxis;
                R_M_L.block<3, 1>(0, 1) = Yaxis;
                R_M_L.block<3, 1>(0, 2) = Zaxis;
                Eigen::Quaterniond Q_M_L(R_M_L);
                Eigen::Vector3d p_M_L = P1;

                iter->second.positionAtCL = p_M_L;
                iter->second.quaternionM2CL = Q_M_L;
                iter->second.rotmatM2L = R_M_L;

                iter->second.isTriangulation = false;
                iter->second.isComputePose = true;

                if (this->showImage_) // 展示Marker坐标系
                {
                    cv::Mat leftImg = this->imageMeasuementQueue_.front().image0;
                    
                    Eigen::Matrix3d R_C_I; // Camera frame to Image frame
                    R_C_I << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
                    Eigen::Vector3d pOri = R_C_I * p_M_L;
                    Eigen::Vector3d pXaxis = this->markerSize_  * R_C_I * Xaxis + pOri;
                    Eigen::Vector3d pYaxis = this->markerSize_  * R_C_I * Yaxis + pOri;
                    Eigen::Vector3d pZaxis = this->markerSize_  * R_C_I * Zaxis + pOri;

                    Eigen::Vector3d pX = this->cameraInfo_[0].K * pXaxis / (pXaxis[2]);
                    Eigen::Vector3d pY = this->cameraInfo_[0].K * pYaxis / (pYaxis[2]);
                    Eigen::Vector3d pZ = this->cameraInfo_[0].K * pZaxis / (pZaxis[2]);
                    Eigen::Vector3d pO = this->cameraInfo_[0].K * pOri / (pOri[2]);
                    cv::line(leftImg, cv::Point2f(pO[0], pO[1]), cv::Point2f(pX[0], pX[1]), cv::Scalar(255, 0, 0), 2, 4);
                    cv::line(leftImg, cv::Point2f(pO[0], pO[1]), cv::Point2f(pY[0], pY[1]), cv::Scalar(255, 0, 0), 2, 8);
                    cv::line(leftImg, cv::Point2f(pO[0], pO[1]), cv::Point2f(pZ[0], pZ[1]), cv::Scalar(255, 0, 0), 2, 16);

                    std::ostringstream ss;
                    ss << "Position: ["<< std::fixed  << std::setprecision(3) << p_M_L[0] << ", " << p_M_L[1] << ", " << p_M_L[2] << "]";
                    cv::putText(leftImg, ss.str(), cv::Point(pO[0]-25, pO[1]-50), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                    Eigen::Vector3d euler = R_M_L.eulerAngles(2,1,0);
                    euler = euler / M_PI * 180;
                    std::ostringstream ss1;
                    ss1 << "Attitude: ["<< std::fixed << std::setprecision(3) << euler[0] << ", " << euler[1] << ", " << euler[2] << "]";
                    cv::putText(leftImg, ss1.str(), cv::Point(pO[0]-25, pO[1]-25), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
                }
            }
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::DisplayImage()
{
    if (this->imageMeasuementQueue_.size() > 0)
    {

        cv::Mat leftImg = this->imageMeasuementQueue_.front().image0;
        cv::Mat rightImg = this->imageMeasuementQueue_.front().image1;
        if (this->showImage_)
        {
            std::ostringstream ss;
            double fps = 1.0 / (this->imageMeasuementQueue_.front().timeStamp - this->preImageTime_);
            this->fps_ = fps;
            ss << std::fixed << "time stamp: " << std::setprecision(2) << this->imageMeasuementQueue_.front().timeStamp << ", fps: " << fps;
            cv::Mat img;
            cv::hconcat(leftImg, rightImg, img);
            cv::putText(img, ss.str(), cv::Point(25, 25), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1);
            this->aviWriter_.write(img);
            cv::imshow("Aruco detector", img);
            cv::waitKey(1);
        }
    }
}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::RejectInvalidMarker()
{
    if (this->markerObservation_.size() != 0)
    {
        for (auto iter = this->markerObservation_.begin(); iter != this->markerObservation_.end(); iter++)
        {
            if(iter->second.isComputePose)
            {
                auto preIter = this->preMarkerObservation_.find(iter->first);
                if(preIter != this->preMarkerObservation_.end())
                {
                    Eigen::Vector3d motionVel = (iter->second.positionAtCL - preIter->second.positionAtCL)/(iter->second.timeStamp - preIter->second.timeStamp);
                    if(motionVel.norm() > this->visionParam_.markerMoveVelThres) // When the marker vel is larger than 2 m/s, this marker is invalid
                    {  
                        iter->second.isComputePose = false;
                        this->isReDetect_ = true;
                        LOG(INFO) << "Reject marker is carried out.";
                    }
                    else
                    {
                        this->preMarkerObservation_.erase(preIter);
                        this->preMarkerObservation_.insert(std::pair<MarkerID, Marker>(iter->first, iter->second));
                    }   
                }            
            }
        }
    }

}
/****
 *
 *
 *
 ****/
void FBUSEKF::VISION::ClearMarkerMap()
{
    this->markerObservation_.erase(this->markerObservation_.begin(), this->markerObservation_.end());
}
