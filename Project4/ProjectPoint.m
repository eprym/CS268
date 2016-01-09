%Written by Konstantine Tsotsos, May 2013
%ktsotsos@cs.ucla.edu, UCLA Vision Lab

function [p d] = ProjectPoint(P)
%Input:  P=(X,Y,Z) in R^3xm pixel coordinates in image for m points
%Note:   
%Output: p in R^2xm 3D coordiantes of m points in camera frame
%Output: d in R^1xm depth of points

% Calibration Information
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
K = [fu 0 cu; 0 fv cv; 0 0 1];
d = P(3,:);
p = P./[d;d;d];
p = K * p;
p = p(1:2,:);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end