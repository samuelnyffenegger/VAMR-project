function [inlier_query_keypoints, corresponding_landmarks, M_W_C2] =  ...
    initialization_KLT(database_image, query_image, K)
% [inlier_query_keypoints, corresponding_landmarks, M_W_C2] =  ...
%     initialization_KLT(database_image, query_image, K)
% establishes keypoint correspondences using Lucas-Kanade tracking, estimates the
% relative pose between frames and triangulates a point cloud of 3D
% landmarks with RANSAC to filter the outliers
% Input:
%   database_image, 1st image --> R = eye(3), t = zeros(3,1)
%   query_image, 2nd image --> M_12 = [R_12|t_12] 
%   K, 3x3 camera matrix
% Output:
%   inlier_query_keypoints, 2xN tracked keypoints with RANSAC telling that
%   these are inliers, N = n_inlier_matches
%   corresponding_landmarks, 3xN 3D world point where i-th point corresponds to
%   i-th inlier_query_keypoints, in world frame

%% calculations 
run('param.m');

% bridge
try
    % launched inside initialization_patch_matching
    database_image = img1;
    query_image = img2;
catch
    % launched outside
end

% get harris keypoints on first (database) image
database_scores = harris(database_image, harris_patch_size, harris_kappa);
database_keypoints = selectKeypoints(database_scores, num_keypoints, nonmaximum_supression_radius); % [y1,y2,...;x1,x2,...]

% track keypoints using Lucas Kanade tracking (KLT)
tracker = vision.PointTracker('MaxBidirectionalError',max_bidirectional_error, ...
    'BlockSize',KLT_patch_size*ones(1,2),'MaxIterations',KLT_max_iterations); 
initialize(tracker,fliplr(database_keypoints'),database_image);
[query_keypoints, point_validity] = step(tracker,query_image);
query_keypoints = fliplr(query_keypoints)';
num_tracked_keypoints = sum(point_validity>0);
tracked_query_keypoints = query_keypoints(:,point_validity);
tracked_database_keypoints = database_keypoints(:,point_validity);

% compute essential matrix in RANSAC fashion
% C1 = database frame, C2 = query frame
[R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, max_num_inliers_history] = ...
    estimateProjectionRANSAC(tracked_database_keypoints, ...
    tracked_query_keypoints, K, n_iterations_matching_RANSAC, pixel_tolerance_RANSAC);
num_tracked_inlier_keypoints = sum(best_inlier_mask>0);
inlier_query_keypoints = tracked_query_keypoints(:,best_inlier_mask);

% homogeneous transformation matrices
M_C2_C1 = [R_C2_C1, t_C2_C1];
M_C1_C2 = [R_C2_C1', -R_C2_C1'*t_C2_C1];
M_W_C2 = M_C1_C2; 

% convert point cloud in C2 frame into C1 (W) frame
P_C1 = M_C1_C2*[P_C2; ones(1,size(P_C2, 2))];
P_W = P_C1;
corresponding_landmarks = P_W; 

% drop points which are behind the camera
drop_mask = find(P_C2(3,:) > 0);
inlier_query_keypoints = inlier_query_keypoints(:,drop_mask);
corresponding_landmarks = corresponding_landmarks(:,drop_mask);

% ground guess
ground_guess = [[0;0],M_W_C2([1,3],4)]; % [x,z]

% talk
if talkative_initialization
    fprintf('keypoints: \n\ttracked = %i, \n\ttracked inliers = %i\n\n', ...
        num_tracked_keypoints,num_tracked_inlier_keypoints)
    eulXYZ = rad2deg(rotm2eul(M_C2_C1(1:3,1:3),'XYZ'));
    turn_arround_y_deg = -eulXYZ(2); % SN: why negative? 
    % fprintf('homogeneous transformation matrix: M_W_C2\n'); disp(M_W_C2);
    fprintf('motion W -> C2: (in world frame)\n\tup: z = %2.2f \n',M_W_C2(3,4))
    fprintf('\tright: x = %2.2f \n\trotation: theta = %2.2f deg\n\n',M_W_C2(1,4),turn_arround_y_deg)
end

% plot overview
plotOverview(query_image, query_keypoints, ...
    tracked_query_keypoints, tracked_database_keypoints, best_inlier_mask, ...
    corresponding_landmarks, M_W_C2, num_tracked_keypoints, ground_guess)

end
