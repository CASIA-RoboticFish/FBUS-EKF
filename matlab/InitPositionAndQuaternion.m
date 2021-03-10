% InitPositionAndQuaternion compute the init position and quaternion of IMU
% frame according to the marker map.
% 
% Input£º                                                                    
%   State: struct of the system state
%   visionMeas: vision measurement. The first row is ID of marker. The 2~4
%   row is position of marker. The 5~8 row is quaternion of marker.
%   markerMap: struct of the information of marker map
%   cameraInfo: struct of the camera parameter
% Output£º
%   State£ºupdated struct of the system state
% Examples£º
%   [State] = InitPositionAndQuaternion(State, visionMeas, markerMap, cameraInfo) 
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

function [State] = InitPositionAndQuaternion(State, visionMeas, markerMap, cameraInfo) 
% Find the vision measurement of nearest marker
idMeas = visionMeas(1, :);
min_id = 0;
min_distance = 10;
for i = 1:length(idMeas)
    distance = norm(visionMeas(2:4, i));
    if(distance < min_distance)
        min_distance = distance;
        min_id = i;
    end
end
posMeas = visionMeas(2:4, min_id);
quatMeas = visionMeas(5:8, min_id);
markerPos = markerMap{idMeas(min_id)+1, 1}.position;
markerQuat = rotmat_to_quaternion(markerMap{idMeas(min_id)+1, 1}.rotation);

% Read camera information
T_IL = cameraInfo.Left_Camera_param.TSC; % The transform matrix from IMU frame{I} to Left Camera frame{L}
R_IL = T_IL(1:3,1:3);
P_IL = -R_IL'*T_IL(1:3,4);
Q_IL = rotmat_to_quaternion(R_IL);

% Compute the quaternion form IMU frame{I} to World frame{G}
Q_MG = markerQuat; % Quaternion form Marker frame{M} to World frame{G}
Q_ML = quatMeas;
Q_IG = quaternion_add(quaternion_add(Q_MG, quaternion_conjugate(Q_ML)), Q_IL);

% Compute the position vector form IMU frame{I} to World frame{G}
R_IG = quaternion_to_rotmat(Q_IG);
R_LI = R_IL';
P_ML = posMeas;
P_MG = markerPos;  % Position form World frame{G} to Marker frame{M} represented at World frame{G}
P_IG = -R_IG*R_LI*P_ML + P_MG - R_IG*P_IL;

% Update the State struct
State.quaternion = Q_IG;
State.position = P_IG;
State.rotateMat = quaternion_to_rotmat(State.quaternion);
State.eulerangle = 180/pi*quaternion_to_eulerangle(State.quaternion);
% State.gravity = State.rotateMat * meanAccel;
State.gravity = [9.8; 0; 0]; % assume the marker is vertical
end
