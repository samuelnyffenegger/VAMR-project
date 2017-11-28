function [R_C2_C1, t_C2_C1] = estimateProjectionRANSAC(p1, p2, K1, K2)
# % TODO: write proper description
% estimateEssentialMatrix_normalized: estimates the essential matrix
% given matching point coordinates, and the camera calibration K in a 
% RANSAC fashion
% Input: 
%  p1, 3xN homogeneous coordinates of 2-D points in image 1
%  p2, 3xN homogeneous coordinates of 2-D points in image 2
%  K1, 3x3 calibration matrix of camera 1
%  K2, 3x3 calibration matrix of camera 2
% Output:
%  E, 3x3 fundamental matrix

%% calculations

% bridge
try
    % launched inside estimateEssentialMatrixRANSAC
    p1 = matched_query_keypoints_homog;
    p2 = matched_database_keypoints_homog;
    K1 = K;
    K2 = K;
catch
    % launched outside
end

warning('no RANSAC, all points taken')

F = fundamentalEightPoint_normalized(p1, p2);

% Compute the essential matrix from the fundamental matrix given K
E = K2'*F*K1;

% RANSAC loop
% pick randomly 8 points
% compute E
% Compute Rots, u3
% disambigue Rots
% triangulate
% reproject
% RANSAC update

end
