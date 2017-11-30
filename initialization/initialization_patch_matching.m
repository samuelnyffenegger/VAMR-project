function [inlier_query_keypoints, corresponding_landmarks] = initialization_patch_matching(query_image, database_image, K)
% [inlier_query_keypoints, corresponding_landmarks] =  ...
% initialization_patch_matching(query_image, database_image, K);
% establishes keypoint correspondences using patch matching, estimates the
% relative pose between frames and triangulates a point cloud of 3D
% landmarks with RANSAC to filter the outliers
% Input:
%   query_image, 2nd image --> M_12 = [R_12|t_12] 
%   database_image, 1st image --> R = eye(3), t = zeros(3,1)
%   K, 3x3 camera matrix
% Output:
%   inlier_query_keypoints, 2xN matched keypoints with RANSAC telling that
%   these are inliers, N = n_inlier_matches
%   corresponding_landmarks, 3xN 3D world point where i-th point corresponds to
%   i-th inlier_query_keypoints

%% calculations
clc

% control parameters
plot_all_matches = true; 
plot_inlier_matches = true;

% parameters 
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;


% bridge
try
    % launched inside initialization_patch_matching
    query_image = img2;
    database_image = img1;
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
n_matched_keypoints = size(matched_database_keypoints,2)

% plot all matches
if plot_all_matches 
    figure(1); clf; 
        imshow(query_image); hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatches(all_matches, query_keypoints, database_keypoints, 2, 'g-')
end

% compute essential matrix in RANSAC fashion

[R_C2_C1, t_C2_C1, best_inlier_mask, max_num_inliers_history] = ...
    estimateProjectionRANSAC(matched_database_keypoints, matched_query_keypoints, K);
R_C2_C1
t_C2_C1

% plot 
figure(1); clf
    subplot(3, 1, 1);
        imshow(query_image);
        hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatches(all_matches, query_keypoints, database_keypoints);
        title('All keypoints and matches');

    subplot(3, 1, 2);
        imshow(query_image);
        hold on;
        plot(matched_query_keypoints(2, (1-best_inlier_mask)>0), ...
            matched_query_keypoints(1, (1-best_inlier_mask)>0), 'rx', 'Linewidth', 2);
        plot(matched_query_keypoints(2, (best_inlier_mask)>0), ...
            matched_query_keypoints(1, (best_inlier_mask)>0), 'gx', 'Linewidth', 2);
        plotMatches(matched_query_mask(best_inlier_mask>0), ...
            matched_query_keypoints(:, best_inlier_mask>0), ...
            database_keypoints);
        hold off;
        title('Inlier and outlier matches');

    subplot(3, 1, 3);
        plot(max_num_inliers_history);
        title('Maximum inlier count over RANSAC iterations.');
                 
                 
%% dummy assignement
if ~exist('inlier_query_keypoints','var'); inlier_query_keypoints = 0; ...
   warning('dummy assignement inlier_query_keypoints');  end
if ~exist('corresponding_landmarks','var'); corresponding_landmarks = 0; ...
   warning('dummy assignement corresponding_landmarks'); end
                 
end


%% snippets
% % indexing to get matched inliers
% n_matched_inlier_keypoints = nnz(inliers_mask)
% inliers_matches = all_matches(all_matches>0).*inliers_mask;
% all_inliers_mask = all_matches > 0;
% all_inliers_mask(all_inliers_mask>0) = inliers_matches;
% all_inliers_matches = all_matches.*all_inliers_mask;



