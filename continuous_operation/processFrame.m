function [S, R_C_W, t_C_W] = processFrame(I,prev_I,prev_S,K)
% Params:
% S.P is 2xN - keypoints
% S.X is 3xN - 3D landmarks
% S.C is 2xK - new candidtate keypoints
% S.F is patch_sizexK - corresponding descriptors at first occurence
% S.T is 12xK - corresponding T at first occurence

run('param.m');

% Localization
KLT = vision.PointTracker('MaxBidirectionalError', 1) % TODO: check out possible options 
initialize(KLT, fliplr(prev_S.P'), prev_I); 
[P_new,point_validity] = step(KLT,I);
S.X=prev_S.X(:, point_validity); % only keep the points that were tracked
S.P=flipud(P_new(point_validity,:)');

% plot all matches
if plot_tracking 
    figure(2);
    clf;
    title('matches tracked keypoints')
    imshow(I); hold on;
    plot(S.P(2, :), S.P(1, :), 'rx', 'Linewidth', 2);
    plotMatches([1:size(S.P,2)], prev_S.P(:,point_validity), S.P, 2, 'g-');
    pause(0.001);
    hold off
end

% localize with P3P and ransac
[R_C_W, t_C_W, max_num_inliers_history] = ransacP3P(S.X,flipud(S.P),K); % looks like ransacP3P nees switched x and y too
T_C_W = [R_C_W, t_C_W];
T_C_W = T_C_W(:);

%% track keypoints

% find harris keypoints and patch descriptors
query_scores = harris(I, harris_patch_size_cont, harris_kappa_cont);
query_keypoints = selectKeypoints(query_scores, num_keypoints_cont, nonmaximum_supression_radius_cont);
query_descriptors = describeKeypoints(I, query_keypoints, descriptor_radius_cont);

database_keypoints = prev_S.C;
database_descriptors = prev_S.F;

if size(database_keypoints,2) == 0
    S.C = query_keypoints;
    S.F = query_descriptors;
    S.T = repmat(T_C_W, 1, size(query_keypoints,2));

else
    % match descriptors 
    all_matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda_cont);
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
      
    if plot_tracking 
    figure(3);
    title('matches new keypoints')
    imshow(I); hold on;
    plot(matched_query_keypoints(2, :), matched_query_keypoints(1, :), 'rx', 'Linewidth', 2);

    plotMatches([1:size(matched_database_keypoints,2)], matched_database_keypoints, matched_query_keypoints, 2, 'g-');
    pause(0.001);
    hold off
    end
    
    % only triangulate points that come from the same image. otherwhise
    % this does not make sense
    unique_transforms = unique(matched_database_transforms', 'rows')';
    
    for i=1:size(unique_transforms,2)
        transform_mask = all(S.T == unique_transforms(:,i),1);
        
        if sum(transform_mask) < min_points
            S.C(:, transform_mask) = [];
            S.F(:, transform_mask) = [];
            S.T(:, transform_mask) = [];
            sprintf('few points, ignore this set') 
            sum(transform_mask)
            continue
        end
        
        % only triangulate points that are not too close to features that
        % already exist
        
        
        sprintf('trangulate new points and check angles')
        sum(transform_mask)
        [R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
        max_num_inliers_history] = estimateProjectionRANSAC(matched_database_keypoints(:,transform_mask), ...
        matched_query_keypoints(:,transform_mask), K, n_iterations, pixel_tolerance);
        t_C2_C1
        R_C2_C1
        
        % rescale translation
        T_C_W_i = reshape(unique_transforms(:,i), 3,4);
        T_W_C_i = [T_C_W_i(1:3,1:3)' -T_C_W_i(1:3,1:3)'*T_C_W_i(1:3,4)];
       
        t_W_C_i = T_W_C_i(1:3,4); % position at first frame
        t_W_C = - R_C_W' * t_C_W; % position now
        norm_t_localization = norm(t_W_C - t_W_C_i);
        norm_t_triangulation = norm(t_C2_C1);
        
        scale_correction = norm_t_localization / norm_t_triangulation;
        
        t_C2_C1 = scale_correction * t_C2_C1;
        P_C2 = P_C2 * scale_correction;
        
        
        if isempty(t_C2_C1) || isempty(R_C2_C1)
            continue
        end
        
        % drop points behind camera
        behind_camera_mask = P_C2(3, :) < 0;
        
         % triangulated points in original coordinate frame 
         P_C1 = R_C2_C1' * P_C2 - R_C2_C1' * t_C2_C1;
         P_C1_in_C2 = P_C2 - t_C2_C1; % points from C1 to P in frame C2.

         angles_deg = acosd(dot(P_C1_in_C2, P_C2) ./ (vecnorm(P_C1_in_C2,2) .* vecnorm(P_C2,2)));

         triangulate_mask = bitand(abs(angles_deg) > alpha_deg, not(behind_camera_mask));
         sprintf('triangulated successfully so many points:')
         sum(triangulate_mask)
         
         % add new triangulated points
         new_points = P_C1(:, triangulate_mask);
         new_points_W = T_W_C_i*[new_points; ones(1,size(new_points,2))];
         S.X = [S.X new_points_W(1:3,:)];
         matched_query_keypoints_i = matched_query_keypoints(:,transform_mask);
         S.P = [S.P matched_query_keypoints_i(:,triangulate_mask)];
         
         if plot_tracking
            figure(2);
            hold on
            new_P = matched_query_keypoints_i(:,triangulate_mask);
            plot(new_P(2,:), new_P(1, :), 'bo', 'Linewidth', 1);
            hold off
         end
            
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