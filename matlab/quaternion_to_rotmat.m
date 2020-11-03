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
function [R] = quaternion_to_rotmat(q)
R = zeros(3);
R(1,1) = q(1)^2+q(2)^2-q(3)^2-q(4)^2;
R(1,2) = 2*(q(2)*q(3)-q(1)*q(4));
R(1,3) = 2*(q(2)*q(4)+q(1)*q(3));
R(2,1) = 2*(q(2)*q(3)+q(1)*q(4));
R(2,2) = q(1)^2-q(2)^2+q(3)^2-q(4)^2;
R(2,3) = 2*(q(3)*q(4)-q(1)*q(2));
R(3,1) = 2*(q(2)*q(4)-q(1)*q(3));
R(3,2) = 2*(q(3)*q(4)+q(1)*q(2));
R(3,3) = q(1)^2-q(2)^2-q(3)^2+q(4)^2;
end
