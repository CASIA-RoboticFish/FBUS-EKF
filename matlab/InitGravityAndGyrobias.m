% InitGravityAndGyrobias compute the gravity vector and gyro bias of IMU
% frame
% 
% Input£º                                                                    
%   imuData: IMU data. First column is timestamp. The 2~4 column is accel
%   data. The 5~7 column is gyro data.
% Output£º
%   gravity£ºgravity vector
%   gyrobias: gyro bias vector
% Examples£º
%   [gravity, gyrobias] = InitGravityAndGyrobias(imuData)
%
% Version 1.0£¬written at 2020.10.01, author: Pengfei Zhang
%
%   An Open-Source, Fiducial-Based, Underwater Stereo Visual-Inertial Localization Method with Refraction Correction
%
%   Author: Pengfei Zhang, 2020
%   Institute of Automation£¬Chinese Academy of Sciences
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

function [gravity, gyrobias] = InitGravityAndGyrobias(imuData)
meanImuData = sum(imuData, 1)/length(imuData);
gravity = -[0; 0; norm(meanImuData(2:4))]; 
gyrobias = meanImuData(5:7)';
end
