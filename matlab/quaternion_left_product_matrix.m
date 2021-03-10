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
function [L] = quaternion_left_product_matrix(q)
qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);
L = qw * eye(4,4);
L(1,2) = -qx;
L(1,3) = -qy;
L(1,4) = -qz;
L(2,1) = qx;
L(2,3) = -qz;
L(2,4) = qy;
L(3,1) = qy;
L(3,2) = qz;
L(3,4) = -qx;
L(4,1) = qz;
L(4,2) = -qy;
L(4,3) = qx;
end

