function [outP, outC] = insertLambertianPlane(I, img1, img2, DIm)
%Input: The original image I, the original book cover img1, the book cover
%that we want to replace with img2, the depth image DIm
%Output: The new points outP generated to represent the object, along with
%their colors outC
%OutP in R^3xm for m points
%OutC in R^3xm for colours of m points

mosaic = sift_mosaic(img1, I, img2, 5000, 2);
[outP outC] = getPointcloud(mosaic, DIm);
end