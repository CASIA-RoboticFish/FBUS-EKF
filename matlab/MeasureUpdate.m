% MeasureUpdate update the system state estimated by ImuUpdate
% according to the current visual measurement and marker map.
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
%   [State] = MeasureUpdate(State, visionMeas, markerMap, cameraInfo)
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
function [State] = MeasureUpdate(State, visionMeas, markerMap, cameraInfo)
% Read camera information
L1 = zeros(4,3);
L1(2,1) = 0.5;
L1(3,2) = 0.5;
L1(4,3) = 0.5;
L2 = eye(4);
L2(2:4,2:4) = -L2(2:4,2:4);
T_IL = cameraInfo.Left_Camera_param.TSC;
R_IL = T_IL(1:3,1:3);
P_IL = -R_IL'*T_IL(1:3,4);
Q_IL = rotmat_to_quaternion(R_IL);

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

% Compute the estimated measurement
posEst = (State.rotateMat*R_IL')'*(markerPos - State.position - State.rotateMat*P_IL);
quatEst = quaternion_add(quaternion_add(Q_IL, quaternion_conjugate(State.quaternion)), markerQuat);

% Compute measurement Jacobian matrix
H = zeros(7,18);
H(1:3,1:3) = -(State.rotateMat*R_IL')';
H(1:3,7:9) = R_IL*vector_to_crossmat(State.rotateMat'*(markerPos-State.position));
H(4:7,7:9) = quaternion_right_product_matrix(markerQuat) * quaternion_left_product_matrix(Q_IL) * L2 * ...
    quaternion_left_product_matrix(State.quaternion) * L1;

if(norm(quatMeas - quatEst) > norm(quatMeas + quatEst))
    quatEst = -quatEst;
    H(4:7,7:9) = -quaternion_right_product_matrix(markerQuat) * quaternion_left_product_matrix(Q_IL) * L2 * ...
    quaternion_left_product_matrix(State.quaternion) * L1;
end

% Compute Kalman gain
K = State.covariance * H' * inv(H*State.covariance*H' + State.measureNoise);

% Compute the state error
err = [posMeas-posEst; quatMeas-quatEst];
err = [posMeas-posEst; 0; 0; 0; 0];
deltaX = K*err;

% Update the State struct
State.position = State.position + deltaX(1:3);
State.velocity = State.velocity + deltaX(4:6);
State.quaternion = quaternion_add(State.quaternion, axisangle_to_quaternion(deltaX(7:9),norm(deltaX(7:9))));
State.quaternion = quaternion_normalize(State.quaternion);
State.accelBias = State.accelBias + deltaX(10:12);
State.gyroBias = State.gyroBias + deltaX(13:15);
State.gravity = State.gravity + deltaX(16:18);

%  Update the state covariance matrix
State.covariance = (eye(18) - K*H) * State.covariance;
State.covariance = (State.covariance + State.covariance')/2;
end