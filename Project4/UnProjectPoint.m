%Written by Konstantine Tsotsos, May 2013
%ktsotsos@cs.ucla.edu, UCLA Vision Lab

function [P] = UnProjectPoint(p, d)
%Input:  p=(i,j) in R^2xm pixel coordinates in image for m points
%Input:  d in R^1xm depth of points in s
%Note:   
%Output: P in R^3xm 3D coordiantes of m points in camera frame

% Calibration Information for this Kinect camera
% Focal Lengths: 
fu = 533.15;
fv = 534.51;
% Principal Point: 
cu = 315.30; 
cv = 245.98;
% Radial (OpenCV Style):
k0 = 0.0728;
k1 = -0.3772;
k4 = 0.4652;
% (tangential) 
k2 = 0.0060;
k3 = -0.0002;

% CS268 HW4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Implement something
P = []; %dummy values
%f = (fu + fv)/2;
K = [fu 0 cu; 0 fv cv; 0 0 1];
for i = 1 : size(p,2)
    pixel_pos = [p(1,i);p(2,i);1];
    cam_pos = inv(K) * pixel_pos;
    world_pos = [cam_pos * d(i)];
    P = [P world_pos];
end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end