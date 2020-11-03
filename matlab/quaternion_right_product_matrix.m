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
function [R] = quaternion_right_product_matrix(q)
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);
R = qw * eye(4,4);
R(1,2) = -qx;
R(1,3) = -qy;
R(1,4) = -qz;
R(2,1) = qx;
R(2,3) = qz;
R(2,4) = -qy;
R(3,1) = qy;
R(3,2) = -qz;
R(3,4) = qx;
R(4,1) = qz;
R(4,2) = qy;
R(4,3) = -qx;
end

