function [S, R_C_W, t_C_W] = processFrame(I,prev_I,prev_S,K)
% Params:
% S.P is 2xN - keypoints
% S.X is 3xN - 3D landmarks
% S.C is 2xK - new candidtate keypoints
% S.F is patch_sizexK - corresponding descriptors at first occurence
% S.T is 12xK - corresponding T at first occurence

% Localization
KLT = vision.PointTracker('MaxBidirectionalError', 1) % TODO: check out possible options 
initialize(KLT, fliplr(prev_S.P'), prev_I); 
[P_new,point_validity] = step(KLT,I);
S.X=prev_S.X(:, point_validity); % only keep the points that were tracked
S.P=flipud(P_new(point_validity,:)');

% plot all matches
plot_tracking = true;
if plot_tracking 
    figure(1); clf; 
        imshow(I); hold on;
        plot(prev_S.P(2, :), prev_S.P(1, :), 'rx', 'Linewidth', 2);
        plotMatches([1:size(S.P,2)], prev_S.P(:,point_validity), S.P, 2, 'g-');
        pause(0.001);
end

% localize with P3P and ransac
[R_C_W, t_C_W, max_num_inliers_history] = ransacP3P(S.X,flipud(S.P),K); % looks like ransacP3P nees switched x and y too
T_C_W = [R_C_W, t_C_W];
T_C_W = T_C_W(:);

%% track keypoints
% parameters 
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 200;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 5;

% find harris keypoints and patch descriptors
query_scores = harris(I, harris_patch_size, harris_kappa);
query_keypoints = selectKeypoints(query_scores, num_keypoints, nonmaximum_supression_radius);
query_descriptors = describeKeypoints(I, query_keypoints, descriptor_radius);

database_keypoints = prev_S.C;
database_descriptors = prev_S.F;

if size(database_keypoints,2) == 0
    S.C = query_keypoints;
    S.F = query_descriptors;
    S.T = repmat(T_C_W, 1, size(query_keypoints,2));

else
    % match descriptors 
    all_matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda);
    matched_query_mask = all_matches > 0;
    matched_query_keypoints = query_keypoints(:,matched_query_mask);
    matched_database_index = all_matches(all_matches > 0);
    matched_database_keypoints = database_keypoints(:,matched_database_index);
    assert(size(matched_database_keypoints,2) == size(matched_query_keypoints,2));

    matched_database_descriptors = database_descriptors(:,matched_database_index);
    matched_database_transforms = prev_S.T(:,matched_database_index);
    
    S.C = matched_database_keypoints;
    S.F = matched_database_descriptors;
    S.T = matched_database_transforms;
         
    
    % Triangulate new points
    alpha_deg = 1;
    n_iterations = 2000;
    pixel_tolerance = 10;
    
    % only triangulate points that come from the same image. otherwhise
    % this does not make sense
    unique_transforms = unique(matched_database_transforms', 'rows')';
    
    for i=1:size(unique_transforms,2)
        transform_mask = all(S.T == unique_transforms(:,i),1);
        
        if sum(transform_mask) < 50
            sprintf('few points, ignore this set') % TODO also remove?
            sum(transform_mask)
            continue
        end
        sprintf('trangulate new points and check angles')
        sum(transform_mask)
        [R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
        max_num_inliers_history] = estimateProjectionRANSAC(matched_database_keypoints(:,transform_mask), ...
        matched_query_keypoints(:,transform_mask), K, n_iterations, pixel_tolerance);
        t_C2_C1
        R_C2_C1
        
        % drop points behind camera
        behind_camera_mask = P_C2(3, :) < 0;
        
         % triangulated points in original coordinate frame 
         P_C1 = R_C2_C1' * P_C2 - R_C2_C1' * t_C2_C1;
         P_C1_in_C2 = P_C2 - t_C2_C1; % points from C1 to P in frame C2.

         angles_deg = acosd(dot(P_C1_in_C2, P_C2) ./ (vecnorm(P_C1_in_C2,2) .* vecnorm(P_C2,2)));

         triangulate_mask = bitand(abs(angles_deg) > alpha_deg, not(behind_camera_mask));
         sprintf('triangulated successfully so many points:')
         sum(triangulate_mask)
         
         T_C_W_i = reshape(unique_transforms(:,i), 3,4);
         T_W_C_i = [T_C_W_i(1:3,1:3)' -T_C_W_i(1:3,1:3)' * T_C_W_i(1:3,4); [0 0 0 1]];
         
         % add new triangulated points
         new_points = P_C1(:, triangulate_mask);
         new_points_W = T_W_C_i*[new_points; ones(1,size(new_points,2))];
         S.X = [S.X new_points_W(1:3,:)];
         matched_query_keypoints_i = matched_query_keypoints(:,transform_mask);
         S.P = [S.P matched_query_keypoints_i(:,triangulate_mask)];
         
         transform_cols = find(transform_mask);
         triangulated_cols = transform_cols(triangulate_mask);
         S.C(:, triangulated_cols) = [];
         S.F(:, triangulated_cols) = [];
         S.T(:, triangulated_cols) = [];      
    end
    
    % and new key points to F,C,T
    if ~isempty(T_C_W) && ~isempty(matched_database_descriptors)
        unmatched_query_keypoints = query_keypoints(:,not(matched_query_mask));
        unmatched_query_descriptors = query_descriptors(:,not(matched_query_mask));
        unmatched_query_transforms = repmat(T_C_W(:), 1, size(unmatched_query_keypoints,2));
        
        S.C=[S.C unmatched_query_keypoints];
        S.F=[S.F unmatched_query_descriptors];
        S.T=[S.T unmatched_query_transforms];
        assert(size(S.C,2) == size(S.F,2));
        assert(size(S.F,2) == size(S.T,2));
    end
    
end

end
