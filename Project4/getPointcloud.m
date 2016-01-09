%Written by Konstantine Tsotsos, May 2013
%ktsotsos@cs.ucla.edu, UCLA Vision Lab

function [P C] = getPointcloud(I, DIm, maxD)
%Input: DIm Depth map (size of image)
%Inpud: I RGB Image
%Input: Max depth
%Note: Throws out bad points and limits depth
%Output Point cloud in R^3xm
%Output RGB colors associated to P in R^3xm

if ~exist('maxD')
    maxD = 2;
end

%Vectorize depth image
d    = DIm(:);
temp = ones(size(DIm(:,:)));

%Set some temporary flags for bad data
temp(d == 0)    = 0;
temp(d > maxD)  = 0;

%Pull out the xy coordinates we want
[x y] = find(temp);

%Re-flag bad data as NAN
d(d == 0)    = NaN;
d(d > maxD)  = NaN;
kill = ~isnan(d);

%Remove bad data
z = d(kill); 

%Get the 3D points
P = UnProjectPoint([x y]',z');

%Get RGB
R = I(:,:,1);
G = I(:,:,2);
B = I(:,:,3);

C(1,:) = R(sub2ind(size(DIm),x,y));
C(2,:) = G(sub2ind(size(DIm),x,y));
C(3,:) = B(sub2ind(size(DIm),x,y));
end