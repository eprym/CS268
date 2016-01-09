function mosaic = sift_mosaic(im1, im2, im3,Maxiter,error)
% SIFT_MOSAIC Demonstrates matching two images using SIFT and RANSAC
%
%   SIFT_MOSAIC demonstrates matching two images based on SIFT
%   features and RANSAC and computing their mosaic.
%
%   SIFT_MOSAIC by itself runs the algorithm on two standard test
%   images. Use SIFT_MOSAIC(IM1,IM2) to compute the mosaic of two
%   custom images IM1 and IM2.

% AUTORIGHTS

if nargin == 0
  im2 = imresize(double(imread('./Data/images/001.png')),1) ;
  im1 = imresize(double(imread('./Data/orange.png')),0.5) ;
  im3 = imresize(double(imread('./Data/blue1.png')),0.5) ;
  Maxiter = 10000;
  error = 2;
end

% make single
im1 = im2single(im1) ;
im2 = im2single(im2) ;
im3 = im2single(im3) ; 

% make grayscale
im1g = rgb2gray(single(mat2gray(im1)));
im2g = rgb2gray(single(mat2gray(im2)));
% if size(im1,3) > 1, im1g = rgb2gray(im1) ; else im1g = im1 ; end
% if size(im2,3) > 1, im2g = rgb2gray(im2) ; else im2g = im2 ; end

% --------------------------------------------------------------------
%                                                         SIFT matches
% --------------------------------------------------------------------

[f1,d1] = vl_sift(im1g);%, 'levels', 8) ;
[f2,d2] = vl_sift(im2g);%, 'levels', 8) ;

[matches, scores] = vl_ubcmatch(d1,d2) ;
matchesT = matches;
%matches(1,:) = matchesT(2,:);
%matches(2,:) = matchesT(1,:);

numMatches = size(matches,2) ;

X1 = f1(1:2,matches(1,:)) ; X1(3,:) = 1 ;
X2 = f2(1:2,matches(2,:)) ; X2(3,:) = 1 ;

% plotpoint(im1*255,X1);
% plotpoint(im2*255,X2);

% --------------------------------------------------------------------
%                                         RANSAC with homography model
% --------------------------------------------------------------------

clear H score ok ;
for t = 1:Maxiter
  % estimate homograpyh
  subset = vl_colsubset(1:numMatches, 4) ;
  A = [] ;
  for i = subset
    A = cat(1, A, kron(X1(:,i)', vl_hat(X2(:,i)))) ;
  end
%   A = [A(1:2,:); A(4:5,:); A(7:8,:); A(10:11,:)];
%   A(2:2:end,:) = -A(2:2:end,:);
  
  if isempty(A)
      t
      continue;
  end
  [U,S,V] = svd(A) ;
  H{t} = reshape(V(:,9),3,3) ;

  % score homography
  X2_ = H{t} * X1 ;
  du = X2_(1,:)./X2_(3,:) - X2(1,:)./X2(3,:) ;
  dv = X2_(2,:)./X2_(3,:) - X2(2,:)./X2(3,:) ;
  ok{t} = (du.*du + dv.*dv) < error*error ;
  score(t) = sum(ok{t}) ;
end

[score, best] = max(score) ;

%[score, best] = min(score) ;
H = H{best} ;
ok = ok{best} ;
finalH = H;

% --------------------------------------------------------------------
%                                                  Optional refinement
% --------------------------------------------------------------------

function err = residual(H)
 u = H(1) * X1(1,ok) + H(4) * X1(2,ok) + H(7) ;
 v = H(2) * X1(1,ok) + H(5) * X1(2,ok) + H(8) ;
 d = H(3) * X1(1,ok) + H(6) * X1(2,ok) + 1 ;
 du = X2(1,ok) - u ./ d ;
 dv = X2(2,ok) - v ./ d ;
 err = sum(du.*du + dv.*dv) ;
end

if exist('fminsearch') == 2
  H = H / H(3,3) ;
  opts = optimset('Display', 'none', 'TolFun', 1e-8, 'TolX', 1e-8) ;
  H(1:8) = fminsearch(@residual, H(1:8)', opts) ;
else
  warning('Refinement disabled as fminsearch was not found.') ;
end

% --------------------------------------------------------------------
%                                                         Show matches
% --------------------------------------------------------------------

dh1 = max(size(im2,1)-size(im1,1),0) ;
dh2 = max(size(im1,1)-size(im2,1),0) ;

% figure(1) ; clf ;
% subplot(2,1,1) ;
% imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;
% o = size(im1,2) ;
% line([f1(1,matches(1,:));f2(1,matches(2,:))+o], ...
%      [f1(2,matches(1,:));f2(2,matches(2,:))]) ;
% title(sprintf('%d tentative matches', numMatches)) ;
% axis image off ;
% 
% subplot(2,1,2) ;
% imagesc([padarray(im1,dh1,'post') padarray(im2,dh2,'post')]) ;
% o = size(im1,2) ;
% line([f1(1,matches(1,ok));f2(1,matches(2,ok))+o], ...
%      [f1(2,matches(1,ok));f2(2,matches(2,ok))]) ;
% title(sprintf('%d (%.2f%%) inliner matches out of %d', ...
%               sum(ok), ...
%               100*sum(ok)/numMatches, ...
%               numMatches)) ;
% axis image off ;
% 
% drawnow ;

% --------------------------------------------------------------------
%                                                               Mosaic
% --------------------------------------------------------------------

t = im3;
im1 = im2;
im2 = t;
H = inv(H);

% box2 = [1  size(im2,2) size(im2,2)  1 ;
%         1  1           size(im2,1)  size(im2,1) ;
%         1  1           1            1 ] ;
% box2_ = inv(H) * box2 ;
% box2_(1,:) = box2_(1,:) ./ box2_(3,:) ;
% box2_(2,:) = box2_(2,:) ./ box2_(3,:) ;

% ur = min([1 box2_(1,:)]):max([size(im1,2) box2_(1,:)]) ;
% vr = min([1 box2_(2,:)]):max([size(im1,1) box2_(2,:)]) ;
ur = 1:size(im1,2);
vr = 1:size(im1,1);

[u,v] = meshgrid(ur,vr) ;
im1_ = vl_imwbackward(im2double(im1),u,v) ;

z_ = H(3,1) * u + H(3,2) * v + H(3,3) ;
u_ = (H(1,1) * u + H(1,2) * v + H(1,3)) ./ z_ ;
v_ = (H(2,1) * u + H(2,2) * v + H(2,3)) ./ z_ ;
im2_ = vl_imwbackward(im2double(im2),u_,v_) ;


%mass = ~isnan(im1_) + ~isnan(im2_) ;
im1_(isnan(im1_)) = 0 ;
im2_(isnan(im2_)) = 0 ;
%mosaic = (im1_ + im2_) ./ mass ;
mosaic = im1_;
mosaic(im2_ > 0) = 0*im2_(im2_ > 0);
mosaic = im1_ + im2_;

figure(2) ; clf ;
imshow(mosaic/255) ; axis image off ;
title('Mosaic') ;

if nargout == 0, clear mosaic ; end

end