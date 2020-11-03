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
function [q] = rotmat_to_quaternion(R)
K = zeros(4,4);
R = R';
K(1,1) = (1/3) * (R(1,1) - R(2,2) - R(3,3));
K(1,2) = (1/3) * (R(2,1) + R(1,2));
K(1,3) = (1/3) * (R(3,1) + R(1,3));
K(1,4) = (1/3) * (R(2,3) - R(3,2));
K(2,1) = (1/3) * (R(2,1) + R(1,2));
K(2,2) = (1/3) * (R(2,2) - R(1,1) - R(3,3));
K(2,3) = (1/3) * (R(3,2) + R(2,3));
K(2,4) = (1/3) * (R(3,1) - R(1,3));
K(3,1) = (1/3) * (R(3,1) + R(1,3));
K(3,2) = (1/3) * (R(3,2) + R(2,3));
K(3,3) = (1/3) * (R(3,3) - R(1,1) - R(2,2));
K(3,4) = (1/3) * (R(1,2) - R(2,1));
K(4,1) = (1/3) * (R(2,3) - R(3,2));
K(4,2) = (1/3) * (R(3,1) - R(1,3));
K(4,3) = (1/3) * (R(1,2) - R(2,1));
K(4,4) = (1/3) * (R(1,1) + R(2,2) + R(3,3));
[V,D] = eig(K);
q = V(:,4)';
q = [q(4); q(1); q(2); q(3)];
end