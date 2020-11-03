% ImuUpdate propogate the estimated IMU state and the covariance of state
% according to the accel and gyro data
% 
% Input£º                                                                    
%   State: struct of the system state
%   accel: measurement of accelerometer, unit: m/s^2
%   gyro: measurement of gyroscope, unit: rad/s
%   dt: the time interval between two IMU measurement
% Output£º
%   State£ºupdated struct of the system state
% Examples£º
%   [State] = ImuUpdate(State, accel, gyro, dt)
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
function [State] = ImuUpdate(State, accel, gyro, dt)
a = accel - State.accelBias;
w = gyro - State.gyroBias;

% Estimate quaternion
dtheta = norm(w*dt);
qT = quaternion_add(State.quaternion, axisangle_to_quaternion(w, dtheta));
qHalfT = quaternion_add(State.quaternion, axisangle_to_quaternion(w, dtheta/2));

% Estimate velocity, 4-th Runge-Kutta method
R0 = State.rotateMat;
RHalfT = quaternion_to_rotmat(qHalfT);
RT = quaternion_to_rotmat(qT);
kv1 = R0 * a + State.gravity;
kv2 = RHalfT * a + State.gravity;
kv3 = kv2;
kv4 = RT * a + State.gravity;
v = State.velocity + dt/6*(kv1+2*kv2+2*kv3+kv4);

% Estimate position
kp1 = State.velocity;
kp2 = State.velocity + kv1*dt/2;
kp3 = State.velocity + kv2*dt/2;
kp4 = State.velocity + kv3*dt/2;
p = State.position + dt/6*(kp1+2*kp2+2*kp3+kp4);

% Compute the covariance matrix
Fx = eye(18);
Fx(1:3,4:6) = eye(3)*dt;
Fx(4:6,7:9) = -State.rotateMat*vector_to_crossmat(a)*dt;
Fx(4:6,10:12) = -State.rotateMat*dt;
Fx(4:6, 16:18) = eye(3)*dt;
Fx(7:9,7:9) = expm(-vector_to_crossmat(w)*dt);
Fx(7:9,13:15) = -eye(3)*dt;
Fi = [zeros(3,12);
    eye(12);
    zeros(3,12);];
P = Fx*State.covariance*Fx' + Fi*State.systemNoise*Fi';

% Update the State struct
State.quaternion = quaternion_normalize(qT);
State.rotateMat = RT;
State.velocity = v;
State.position = p;
State.covariance = P;
State.covariance = (State.covariance + State.covariance')/2;
end

