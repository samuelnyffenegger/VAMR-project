function F = estimateFundamentalMatrix_normalized(p1, p2, NumTrials, DistanceThreshold)
% F = fundamentalEightPoint_normalized(p1, p2); 
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K
% Input: 
%   p1, 3xN homogeneous coordinates of 2-D points in image 1
%   p2, 3xN homogeneous coordinates of 2-D points in image 2
%   NumTrails, number of RANSAC iteration
%   DistanceThreshold, threshold for RANSAC
% Output:
%   F, 3x3 fundamental matrix

% Normalize each set of points so that the origin
% is at centroid and mean distance from origin is sqrt(2).
[x1_nh,T1] = normalise2dpts(p1);
[x2_nh,T2] = normalise2dpts(p2);

% get image coordinates
x1_nh_ = [x1_nh(1,:);x1_nh(2,:)]./x1_nh(3,:);
x2_nh_ = [x2_nh(1,:);x2_nh(2,:)]./x2_nh(3,:);

% Linear solution
F = estimateFundamentalMatrix(x1_nh_',x2_nh_','Method','RANSAC',...
    'NumTrials',NumTrials,'DistanceThreshold',DistanceThreshold)

% Undo the normalization
F = (T2.') * F * T1;

end

