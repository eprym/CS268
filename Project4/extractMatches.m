function [ matches1, matches2 ] = extractMatches( img1, img2 )
% extract the correspondent inliers for the ICP procedure

img1 = im2single(img1/255);
img2 = im2single(img2/255);
img_gray1 = rgb2gray(img1);
img_gray2 = rgb2gray(img2);
[f1, d1] = vl_sift(img_gray1);
[f2, d2] = vl_sift(img_gray2);
[matches, scores] = vl_ubcmatch(d1,d2,6.0);
size(matches,2)
matches1 = f1(1:2,matches(1,:));
matches2 = f2(1:2,matches(2,:));
matches1 = round(matches1);
matches2 = round(matches2);

end

