function [ P C ] = getPointCloudInlier( I, DIm, inlier )
%   generate the point cloud only with the inliers

z = [];
for i = 1:size(inlier,2)
    z = [z DIm(inlier(2,i), inlier(1,i))];
end


P = UnProjectPoint(inlier, z);

R = I(:,:,1);
G = I(:,:,2);
B = I(:,:,3);

C(1,:) = R(sub2ind(size(DIm),inlier(2,:),inlier(1,:)));
C(2,:) = G(sub2ind(size(DIm),inlier(2,:),inlier(1,:)));
C(3,:) = B(sub2ind(size(DIm),inlier(2,:),inlier(1,:)));
end

