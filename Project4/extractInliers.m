function [ inlier1, inlier2 ] = extractInliers( img1, img2 )
% extract the correspondent inliers for the ICP procedure
img1 = im2single(img1/255);
img2 = im2single(img2/255);
img_gray1 = rgb2gray(img1);
img_gray2 = rgb2gray(img2);
[f1, d1] = vl_sift(img_gray1);
[f2, d2] = vl_sift(img_gray2);
[matches, scores] = vl_ubcmatch(d1,d2,2.5);
% figure(1);
% clf;
% imagesc(cat(2, img1, img2));
% x1 = f1(1, matches(1,:));
% x2 = f2(1, matches(2,:)) + size(img1,2);
% y1 = f1(2, matches(1,:));
% y2 = f2(2,matches(2,:));
% 
% hold on;
% h = line([x1;x2], [y1;y2]);
% set(h,'linewidth', 1, 'color', 'b') ;

numMatches = size(matches,2);
threshold = 4.0;
clear H score ok ;
X1 = f1(1:2,matches(1,:)) ; X1(3,:) = 1 ;
X2 = f2(1:2,matches(2,:)) ; X2(3,:) = 1 ;
for t = 1:2000
  % estimate homograpyh
  subset = vl_colsubset(1:numMatches, 4) ;
  A = [] ;
  for i = subset
    A = cat(1, A, kron(X1(:,i)', vl_hat(X2(:,i)))) ;
  end
  %A = [A(1:2,:); A(4:5,:); A(7:8,:); A(10:11,:)];
  %A(2:2:end,:) = -A(2:2:end,:);
  
  [U,S,V] = svd(A) ;
  H{t} = reshape(V(:,9),3,3) ;

  % score homography
  X2_ = H{t} * X1 ;
  du = X2_(1,:)./X2_(3,:) - X2(1,:)./X2(3,:) ;
  dv = X2_(2,:)./X2_(3,:) - X2(2,:)./X2(3,:) ;
  ok{t} = (du.*du + dv.*dv) < threshold * threshold ;
  score(t) = sum(ok{t}) ;
end

[score, best] = max(score) ;

%[score, best] = min(score) ;
H = H{best} ;
ok = ok{best} ;

% figure(1);
% clf;
% imagesc(cat(2, img1, img2));
% x1 = X1(1,ok);
% x2 = X2(1,ok) + size(img1,2);
% y1 = X1(2,ok);
% y2 = X2(2,ok);
% 
% hold on;
% h = line([x1;x2], [y1;y2]);
% set(h,'linewidth', 1, 'color', 'b') ;

inlier1 = round([X1(1,ok);X1(2,ok)]);
inlier2 = round([X2(1,ok);X2(2,ok)]);

end
