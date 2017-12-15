function [inlier_query_keypoints, corresponding_landmarks, M_W_C2] =  ...
    initialization_patch_matching(database_image, query_image, K)
% [inlier_query_keypoints, corresponding_landmarks, M_W_C2] =  ...
% initialization_patch_matching(query_image, database_image, K);
% establishes keypoint correspondences using patch matching, estimates the
% relative pose between frames and triangulates a point cloud of 3D
% landmarks with RANSAC to filter the outliers
% Input:
%   database_image, 1st image --> R = eye(3), t = zeros(3,1)
%   query_image, 2nd image --> M_12 = [R_12|t_12] 
%   K, 3x3 camera matrix
% Output:
%   inlier_query_keypoints, 2xN matched keypoints with RANSAC telling that
%   these are inliers, N = n_inlier_matches
%   corresponding_landmarks, 3xN 3D world point where i-th point corresponds to
%   i-th inlier_query_keypoints, in world frame

%% parameters 
% control parameters
plot_all_matches = true; 
talkative = true;

% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 10;

% RANSAC essential matrix
n_iterations = 1000;
pixel_tolerance = 2; 

% calculations

% bridge
try
    % launched inside initialization_patch_matching
    database_image = img1;
    query_image = img2;
catch
    % launched outside
end


% find harris keypoints and patch descriptors
query_scores = harris(query_image, harris_patch_size, harris_kappa);
query_keypoints = selectKeypoints(query_scores, num_keypoints, nonmaximum_supression_radius);
query_descriptors = describeKeypoints(query_image, query_keypoints, descriptor_radius);

database_scores = harris(database_image, harris_patch_size, harris_kappa);
database_keypoints = selectKeypoints(database_scores, num_keypoints, nonmaximum_supression_radius);
database_descriptors = describeKeypoints(database_image, database_keypoints, descriptor_radius);


% match descriptors 
all_matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda);
matched_query_mask = all_matches > 0;
matched_query_keypoints = query_keypoints(:,matched_query_mask);
matched_database_index = all_matches(all_matches > 0);
matched_database_keypoints = database_keypoints(:,matched_database_index);
assert(size(matched_database_keypoints,2) == size(matched_query_keypoints,2));
n_matched_keypoints = size(matched_database_keypoints,2);

% compute essential matrix in RANSAC fashion
% C1 = database frame, C2 = query frame
[R_C2_C1, t_C2_C1, P_C2, best_inlier_mask] = ...
    estimateProjectionRANSAC(matched_database_keypoints, ...
    matched_query_keypoints, K, n_iterations, pixel_tolerance);
n_matched_inlier_keypoints = sum(best_inlier_mask>0);
inlier_query_keypoints = matched_query_keypoints(:,best_inlier_mask);

% drop points which are behind the camera
P_C2 = P_C2(:,P_C2(3,:) > 0);

% homogeneous transformation matrices
M_C2_C1 = [R_C2_C1, t_C2_C1];
M_C1_C2 = [R_C2_C1', -R_C2_C1'*t_C2_C1];
M_W_C2 = M_C1_C2; 

% convert point cloud in C2 frame into C1 (W) frame
P_C1 = M_C1_C2*[P_C2; ones(1,size(P_C2, 2))];
P_W = P_C1;
corresponding_landmarks = P_W; 

% number of tracked landmarks
n_tracked_landmarks = n_matched_keypoints;

% ground guess
ground_guess = [[0;0],M_W_C2([1,3],4)]; % [x,z]

% talk
if talkative
    fprintf('keypoints: \n\tmatched = %i, \n\tmatched inliers = %i\n\n', ...
        n_matched_keypoints,n_matched_inlier_keypoints)
    eulXYZ = rad2deg(rotm2eul(M_C2_C1(1:3,1:3),'XYZ'));
    turn_arround_y_deg = -eulXYZ(2); % SN: why negative? 
    % fprintf('homogeneous transformation matrix: M_W_C2\n'); disp(M_W_C2);
    fprintf('motion W -> C2: (in world frame)\n\tup: z = %2.2f \n',M_W_C2(3,4))
    fprintf('\tright: x = %2.2f \n\trotation: theta = %2.2f deg\n\n',M_W_C2(1,4),turn_arround_y_deg)
end

% plot overview
plotOverview(query_image, query_keypoints, ...
    matched_query_keypoints, matched_database_keypoints, best_inlier_mask, ...
    corresponding_landmarks, M_W_C2, n_tracked_landmarks, ground_guess)

% %% plot 
% 
% % plot all inlier matches
% if plot_all_matches 
%     figure(1); clf
%         imshow(query_image); hold on;
%         plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
%         plotMatchedKeypoints(matched_query_keypoints(:,best_inlier_mask), ...
%             matched_database_keypoints(:,best_inlier_mask), 2, 'g-')
%         plotMatchedKeypoints(matched_query_keypoints(:,not(best_inlier_mask)), ...
%             matched_database_keypoints(:,not(best_inlier_mask)), 1, 'c-')
% end
% 
% 
% % Visualize the 3-D scene
% max_visual_distance = 25;
% landmarks_mask = (sqrt(sum(P_W-mean(P_W,2)).^2) < max_visual_distance);
% 
% figure(2); clf;
%     query_cam_W = M_W_C2(1:3,4);
%     plot3(corresponding_landmarks(1,landmarks_mask), corresponding_landmarks(2,landmarks_mask), ...
%          corresponding_landmarks(3,landmarks_mask),'kx','LineWidth',2);
%     hold on; 
%     plot3([0,query_cam_W(1)],[0,query_cam_W(2)],[0,query_cam_W(3)],'bx-')
%     plotCoordinateFrame(eye(3),zeros(3,1), 10);
% 
%     plotCoordinateFrame(M_W_C2(1:3,1:3),M_W_C2(1:3,4), 10);
% 
%     xlabel('x'); ylabel('y'), zlabel('z');
%     axis equal
%     rotate3d on;
%     grid on
%     view([0,0])
%     title('Trajectory of last 20 frames and landmarks')
                 
end
