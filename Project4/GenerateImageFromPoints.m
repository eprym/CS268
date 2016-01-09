%Written by Konstantine Tsotsos, May 2013
%ktsotsos@cs.ucla.edu, UCLA Vision Lab

function [I] = GenerateImageFromPoints(P,C)
%Input:  P   in R^3xm 3D points
%Input:  C   in R^3xm RGB intensities for points
%Output: I   in R^480x640 image generated from the above points

%Project the points to get 2D coordinates and depths
[p, d] = ProjectPoint(P);
%Pull out the non-integer pixel coordinates and adjust them as you see fit
p = (round(p)); % The dumb and easy solution of rounding!


%Do some simple Z-buffering - speed this up if you want
%Get rid of all of the pixels that may still be invalid
% Simple check to makes sure that the pixels are inside the picture frame
valid_indicies = (0 < p(1,:) & p(1,:) <= 480 & 0 < p(2,:) & p(2,:) <= 640);
% Filter the points and associated colors and depths appropriately
p = p(:, valid_indicies);
d = d(valid_indicies);
C = C(:, valid_indicies);

pts = cat(1, p, d); 
[pts, sorted_indicies] = sortrows(pts', [1 2 -3]);
pts = pts'; 
C = C(:, sorted_indicies); 

diff_p = diff(pts')'; 
desired_indicies = [(diff_p(1, :) ~= 0 | diff_p(2, :) ~= 0 )];
pts = pts(:, desired_indicies);
C = C(:, desired_indicies);

I_Rvals = zeros([480 640]);
I_Gvals = zeros([480 640]);
I_Bvals = zeros([480 640]);
C_Red   = C(1,:);
C_Blue  = C(2,:);
C_Green = C(3,:);

pixel_locs = sub2ind(size(I_Rvals), pts(1, :), pts(2, :));

I_Rvals(pixel_locs) = C_Red;
I_Gvals(pixel_locs) = C_Blue;
I_Bvals(pixel_locs) = C_Green;

% Compose the final image
I = zeros([480 640 3]);
I(:,:,1) = I_Rvals;
I(:,:,2) = I_Gvals;
I(:,:,3) = I_Bvals;
end