%
%   An Open-Source, Fiducial-Based, Underwater Stereo Visual-Inertial Localization Method with Refraction Correction
%
%   Author: Pengfei Zhang, 2020
%   Institute of Automation，Chinese Academy of Sciences
%   
%    Copyright (C) <2020>  <Pengfei Zhang>
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <https://www.gnu.org/licenses/>.
%

clc;
clear all;
close all;
%% 用户定义
%% User-defined parameter
% 滤波器常数
% Filter constant
% 视觉测量噪声强度
% Visual measurement noise internsity
POSITION_NOISE_INTENSITY = 0.01;
QUATERNION_NOISE_INTENSITY = 0.01;
% IMU测量噪声强度
% IMU measurement noise intensity
ACCEL_BIAS_NOISE_INTENSITY = 0.001; % sigma_ba
GYRO_BIAS_NOISE_INTENSITY = 0.0001; % sigma_bg
ACCEL_NOISE_INTENSITY = 0.001; % sigma_a_c
GYRO_NOISE_INTENSITY = 0.0001; % sigma_g_c



%% 初始化
%% Initialization
% 数据路径
% File path of dataset
datasetPath = 'dataset/waterdata/dataset-06'; % underwater data
datasetPath = 'dataset/landdata/dataset-02'; % air data
% IMU数据读取
% Read IMU data
imudataOri = load(strcat(datasetPath,'/imu.txt'));
imudataOri(:,5:7) = imudataOri(:,5:7);
imudata = imudataOri;
% 图像数据读取
% Read visual measurement
imgdataOri = load(strcat(datasetPath,'/image.txt'));
imgdata = imgdataOri;
measEuler = [];
for i = 1:length(imgdata)
    measEuler = [measEuler; 180/pi*quaternion_to_eulerangle(imgdata(i,6:9))'];
end
% C++代码计算得到的系统状态
% Read filtered results of C++ implementation
fusiondata = load(strcat(datasetPath,'/fusion.txt'));
% 相机参数读取
% Read camera parameter
cameraInfo = YAML.read('config/camerainfo.yml');
cameraInfo.Left_Camera_param.TSC = [-1,0,0,0;0,-1,0,0;0,0,1,0;0,0,0,1]* cameraInfo.Left_Camera_param.TSC;
% Marker地图读取
% Read marker map
MarkerMap = GetMarkerMap();
% 系统状态初始化
% Initialize system state struct
State.position = zeros(3,1);
State.quaternion = zeros(4,1);
State.eulerangle = zeros(3,1);
State.velocity = zeros(3,1);
State.velocity = zeros(3,1);
State.accelBias = zeros(3,1);
State.gyroBias = zeros(3,1);
State.gravity = zeros(3,1);
State.rotateMat = zeros(3,3);
State.covariance = eye(18,18);
State.systemNoise = eye(12,12);
State.measureNoise = eye(7,7);
% 系统状态协方差矩阵初始化
% Initialize system covariance matrix
positionCov = 0.0001;
State.covariance(1:3, 1:3) = positionCov * eye(3,3);
velocityCov = 0.1;
State.covariance(4:6, 4:6) = velocityCov * eye(3,3);
orientationCov = 0.0001;
State.covariance(7:9, 7:9) = orientationCov * eye(3,3);
accelBiasCov = 0.001;
State.covariance(10:12, 10:12) = accelBiasCov * eye(3,3);
gyroBiasCov = 0.001;
State.covariance(13:15, 13:15) = gyroBiasCov * eye(3,3);
gravityCov = 100;
State.covariance(16:18, 16:18) = gravityCov * eye(3,3);
State.covariance = State.covariance;
% 系统噪声矩阵初始化
% Initialize system noise matrix
State.systemNoise(1:3, 1:3) = ACCEL_BIAS_NOISE_INTENSITY* eye(3,3);
State.systemNoise(4:6, 4:6) = GYRO_BIAS_NOISE_INTENSITY * eye(3,3);
State.systemNoise(7:9, 7:9) = ACCEL_NOISE_INTENSITY * eye(3,3);
State.systemNoise(10:12, 10:12) = GYRO_NOISE_INTENSITY * eye(3,3);
State.systemNoise = State.systemNoise;
% 系统量测噪声矩阵初始化
% Initialize measurement noise matrix
State.measureNoise(1:3, 1:3) = POSITION_NOISE_INTENSITY * eye(3,3);
State.measureNoise(4:7, 4:7) = QUATERNION_NOISE_INTENSITY * eye(4,4);
State.measureNoise =State.measureNoise;



%% 初始化重力矢量和陀螺仪Bias
%% Compute the gravity vector and gyro bias of IMU frame
[State.gravity, State.gyroBias] = InitGravityAndGyrobias(imudata(1:500,:));



%% 初始化相机的位置和姿态
%% Compute the init position and quaternion of IMU frame according to the marker map.
firstImgDataTime = imgdata(1,1);
for i = 1:length(imudata)
    if(imudata(i,1) > firstImgDataTime)
        break;
    end
end
imuDataIdx = i;
visionMeas = imgdata(1,2:9)';
[State] = InitPositionAndQuaternion(State, visionMeas, MarkerMap, cameraInfo);



%% 记录数据的变量初始化
%% Initialize the record variable
Time = []; %
EKFResults = [];
VisionOnlyResults = [];
VisionOnlyResultsFilter = [];



%% EKF主循环
%% Main loop of EKF
preImgTime = 0;
curImgTime = 0;
figure(1)
nImgData = 1;
while(nImgData < length(imgdata))
    
    %% 读取当前时刻的视觉数据
    %% Read current visual measurement
    nImgDataTemp = nImgData + 1;
    while(imgdata(nImgDataTemp,1) == imgdata(nImgData,1))
        nImgDataTemp = nImgDataTemp+1;
        if(nImgDataTemp>length(imgdata))
            break;
        end
    end
    curImgTime = imgdata(nImgData,1);
    visionMeas = imgdata(nImgData:(nImgDataTemp-1),2:9)'; % 视觉测量数据
    nImgData = nImgDataTemp;
    
    %% 判断是否需要重置系统状态
    %% Determine if the system state needs to be reset
    if(curImgTime-preImgTime > 0.1 && preImgTime ~= 0)  % 如果这一次视觉检测结果相比于前一次间隔时间过长，就需要重置系统状态
        State = ResetState(State, visionMeas, MarkerMap, cameraInfo);
        preImgTime = curImgTime;
    else
        
        %% IMU时间更新
        %% IMU time update
        preImuTime = imudata(imuDataIdx-1,1);
        for j = imuDataIdx:length(imudata)
            if(imudata(j,1) > curImgTime)
                break;
            end
            if(imudata(j,1) < preImgTime)
                preImuTime = imudata(j,1);
                continue;
            end
            curImuTime = imudata(j,1);
            dt = curImuTime - preImuTime;
            preImuTime = curImuTime;
            accel = imudata(j,2:4)';
            gyro = imudata(j,5:7)';
            [State] = ImuUpdate(State, accel, gyro, dt); % IMU更新
        end
        imuDataIdx = j;
        preImgTime = curImgTime;
        
        %% 视觉量测更新
        %% Vision measurement update
        [State] = MeasureUpdate(State, visionMeas, MarkerMap, cameraInfo);
    end
    
    %% 记录数据
    %% Record data
    Time = [Time; curImgTime];
    ekfState = [State.position; State.velocity; State.quaternion; State.accelBias; State.gyroBias; State.gravity; 180/pi*quaternion_to_eulerangle(State.quaternion)];
    EKFResults = [EKFResults, ekfState];
    % 计算纯视觉估计的结果
    % Compute the vision-only results
    [pos, quat, rotmat] = ComputeVisionOnlyResults(State, visionMeas, MarkerMap, cameraInfo);
    visonResults = [pos; quat; 180/pi*quaternion_to_eulerangle(quat)];
    VisionOnlyResults = [VisionOnlyResults, visonResults];

end




%% 绘制系统状态
%% Plot system state
figure(1)
PlotState(Time, EKFResults', VisionOnlyResults')

%% 绘制运动轨迹
%% Plot trajectory
figure(2)
plot3(EKFResults(3,:), EKFResults(2,:), -EKFResults(1,:), 'r')
hold on
plot3(VisionOnlyResults(3,:), VisionOnlyResults(2,:), -VisionOnlyResults(1,:), 'b')
hold on
axis equal

%% 绘制IMU数据
%% Plot IMU data
figure(3)
subplot(2,1,1)
plot(imudata(:,1), imudata(:,2), 'r')
hold on
plot(imudata(:,1), imudata(:,3), 'g')
hold on
plot(imudata(:,1), imudata(:,4), 'b')
hold on
plot(imudataOri(:,1), imudataOri(:,2), 'r--')
hold on
plot(imudataOri(:,1), imudataOri(:,3), 'g--')
hold on
plot(imudataOri(:,1), imudataOri(:,4), 'b--')
hold on
subplot(2,1,2)
plot(imudata(:,1), imudata(:,5), 'r')
hold on
plot(imudata(:,1), imudata(:,6), 'g')
hold on
plot(imudata(:,1), imudata(:,7), 'b')
hold on
plot(imudataOri(:,1), imudataOri(:,5), 'r--')
hold on
plot(imudataOri(:,1), imudataOri(:,6), 'g--')
hold on
plot(imudataOri(:,1), imudataOri(:,7), 'b--')
hold on









