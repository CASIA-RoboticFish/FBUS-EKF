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
function [] = PlotState(Time, EKFResults, VisionOnlyResults)
%% 位置
%% Position
subplot(6,3,1)
plot(Time, VisionOnlyResults(:,1), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,1), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('x-position (m)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,2)
plot(Time, VisionOnlyResults(:,2), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,2), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('y-position (m)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,3)
plot(Time, VisionOnlyResults(:,3), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,3), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('z-position (m)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

%% 速度
%% Velocity
subplot(6,3,4)
plot(Time, EKFResults(:,4), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('x-velocity (m/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,5)
plot(Time, EKFResults(:,5), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('y-velocity (m/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,6)
plot(Time, EKFResults(:,6), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('z-velocity (m/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

%% 欧拉角
%% Euler angle
subplot(6,3,7)
plot(Time, VisionOnlyResults(:,8), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,20), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('roll (^\circ)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,8)
plot(Time, VisionOnlyResults(:,9), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,21), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('pitch (^\circ)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,9)
plot(Time, VisionOnlyResults(:,10), 'g', 'LineWidth', 2)
hold on
plot(Time, EKFResults(:,22), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('yaw (^\circ)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

%% 加速度漂移
%% Accel bias
subplot(6,3,10)
plot(Time, EKFResults(:,11), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('x-accel-bias (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,11)
plot(Time, EKFResults(:,12), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('y-accel-bias (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,12)
plot(Time, EKFResults(:,13), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('z-accel-bias (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

%% 陀螺仪漂移
%% Gyro bias
subplot(6,3,13)
plot(Time, EKFResults(:,14), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('x-gyro-bias (rad/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,14)
plot(Time, EKFResults(:,15), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('y-gyro-bias (rad/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,15)
plot(Time, EKFResults(:,16), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('z-gyro-bias (rad/s)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

%% 重力
%% Gravity
subplot(6,3,16)
plot(Time, EKFResults(:,17), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('x-gravity (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,17)
plot(Time, EKFResults(:,18), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('y-gravity (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

subplot(6,3,18)
plot(Time, EKFResults(:,19), 'r', 'LineWidth', 2)
hold on
grid on
xlabel('t (s)','FontName','Arial')
ylabel('z-gravity (m/s^2)','FontName','Arial')
set(gca,'FontName','Arial','GridLineStyle','--','GridAlpha',0.5,'FontWeight','bold','LineWidth',1);

end