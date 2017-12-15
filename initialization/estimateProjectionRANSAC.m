function [R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
    max_num_inliers_history] = estimateProjectionRANSAC(matched_database_keypoints, ...
    matched_query_keypoints, K, n_iterations, pixel_tolerance)
% [R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
%    max_num_inliers_history] = estimateProjectionRANSAC(matched_database_keypoints, ...
%    matched_query_keypoints, K, n_iterations, pixel_tolerance);
% estimates the relative position and orientation giving keypoints on two
% images, frame C1 corresponds to the first (query) image, where frame C2 
% corresponds to the second (database) image. Outliers are removed in a 
% RANSAC fashion
% Input: 
%   matched_database_keypoints (C1), 2xN coordinates of 2-D points in image 1, M1 = [I,0]_C1
%   matched_query_keypoints (C2), 2xN coordinates of 2-D points in image 2
%   K, 3x3 calibration matrix
%   n_iterations, number of RANSAC iterations
%   pixel_tolerance, tolerance in pixel for reprojected triangulated points
% Output:
%   R_C2_C1, rotaion matrix frame C1 in C2
%   t_C2_C1, translation vector frame C1 in C2
%   P_C2, 3xN real world point cloud of triangulated inliers in query frame (C2)
%   best_inlier_mask, 1xN, mask whether matched keypoint is an RANSAC inlier (1)
%   max_num_inliers_history, maximum of number of inliers


%% calculations

% control parameters
plot_sample_keypoints = false;
talkative = false;


% parameters TODO: add those parameters to a central place
if ~exist('n_iterations','var'); n_iterations = 500; end
if ~exist('pixel_tolerance','var'); pixel_tolerance = 5; end
k = 8; 

% Initialize RANSAC
matched_query_keypoints_uv = flipud(matched_query_keypoints); % (row, col) to (u, v)
matched_database_keypoints_uv = flipud(matched_database_keypoints); % (row, col) to (u, v)
n_matched_keypoints = size(matched_database_keypoints_uv,2);
best_inlier_mask = zeros(1, n_matched_keypoints);
max_num_inliers_history = zeros(1, n_iterations);
max_num_inliers = 0;
min_inlier_count = k; 

% RANSAC looop
for i = 1:n_iterations
    % randomly pick k points 
    [sample_database_keypoints, idx] = datasample(matched_database_keypoints_uv, k, 2, 'Replace', false);
    sample_query_keypoints = matched_query_keypoints_uv(:,idx);

    % plot sampled keypoints
    if plot_sample_keypoints
        figure(1);
        plot(matched_query_keypoints_uv(2, :), matched_query_keypoints_uv(1, :), 'rx', 'Linewidth', 2);
        plot(sample_query_keypoints(2, :), sample_query_keypoints(1, :), 'bx', 'Linewidth', 2);
    end
    
    % calculate fundamental matrix
    sample_database_keypoints_homog = [sample_database_keypoints; ones(1,k)];
    sample_query_keypoints_homog = [sample_query_keypoints; ones(1,k)];
    F_guess = fundamentalEightPoint_normalized(sample_database_keypoints_homog, ...
        sample_query_keypoints_homog);
    
    % calculate, decompose and disambiguete essential matrix
    E_guess = K'*F_guess*K;
    [Rots,u3] = decomposeEssentialMatrix(E_guess);
    [R_C2_C1_guess,t_C2_C1_guess] = disambiguateRelativePose(Rots,u3, ...
        sample_database_keypoints_homog, sample_query_keypoints_homog,K,K);

    % Triangulate a point cloud using the guess transformation (R,t)
    M_database = K * eye(3,4);
    M_query = K * [R_C2_C1_guess, t_C2_C1_guess];
    matched_query_keypoints_homog = [matched_query_keypoints_uv; ones(1,size(matched_query_keypoints_uv,2))];
    matched_database_keypoints_homog = [matched_database_keypoints_uv; ones(1,size(matched_database_keypoints_uv,2))];
    P_guess_C1 = linearTriangulation(matched_database_keypoints_homog, ...
        matched_query_keypoints_homog, M_database, M_query);
    
    % reproject points
    P_guess_C2 = (R_C2_C1_guess * P_guess_C1(1:3,:)) + ... 
        repmat(t_C2_C1_guess,[1 size(P_guess_C1, 2)]); 
    projected_points = projectPoints(P_guess_C2(1:3,:), K); 
    
    % count inliers 
    difference = matched_query_keypoints_uv - projected_points;
    errors = sum(difference.^2, 1);
    inlier_mask = errors < pixel_tolerance^2;
    n_inliers = nnz(inlier_mask); 

    % RANSAC update
    if n_inliers > max_num_inliers && ...
            n_inliers >= min_inlier_count
        max_num_inliers = n_inliers;         
        best_inlier_mask = inlier_mask;
        if talkative
           fprintf(['n_inliers(',num2str(i),') = ',num2str(max_num_inliers),'\n']) 
        end
    end
    max_num_inliers_history(i) = max_num_inliers;
end

% final point cloud triangulation and pose estimation using inliers
if max_num_inliers == 0
    warning('RANSAC did not find enough inliers');
    R_C2_C1 = [];
    t_C2_C1 = [];
    P_C2 = [];
else
    % calculate fundamental matrix
    inlier_database_keypoints_homog = [matched_database_keypoints_uv(:,best_inlier_mask); ones(1,max_num_inliers)];
    inlier_query_keypoints_homog = [matched_query_keypoints_uv(:,best_inlier_mask); ones(1,max_num_inliers)];
    F_C2_C1 = fundamentalEightPoint_normalized(inlier_database_keypoints_homog, ...
        inlier_query_keypoints_homog);
    
    % calculate, decompose and disambiguete essential matrix
    E_C2_C1 = K'*F_C2_C1*K;
    [Rots,u3] = decomposeEssentialMatrix(E_C2_C1);
    [R_C2_C1,t_C2_C1] = disambiguateRelativePose(Rots,u3, ...
        inlier_database_keypoints_homog, inlier_query_keypoints_homog,K,K);
    
    % Triangulate a point cloud using the final transformation (R,t) and all inliers
    M_database = K * eye(3,4);
    M_query = K * [R_C2_C1, t_C2_C1];
    inlier_database_keypoints_homog = matched_database_keypoints_homog(:,best_inlier_mask);
    inlier_query_keypoints_homog = matched_query_keypoints_homog(:,best_inlier_mask);
    P_C1 = linearTriangulation(inlier_database_keypoints_homog, ...
        inlier_query_keypoints_homog, M_database, M_query);
    P_C2 = (R_C2_C1 * P_C1(1:3,:)) + repmat(t_C2_C1,[1 size(P_C1, 2)]); 

end



%% dummy assignement
if ~exist('R_C2_C1','var'); R_C2_C1 = eye(3); warning('dummy assignement R_C2_C1'); end
if ~exist('t_C2_C1','var'); t_C2_C1 = zeros(3,1); warning('dummy assignement t_C2_C1');  end

end
