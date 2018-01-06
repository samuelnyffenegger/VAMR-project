function [S, R_C_W, t_C_W] = processFrame(I,prev_I,prev_S,K)
% Params:
% S.P is 2xN - keypoints
% S.X is 3xN - 3D landmarks
% S.C is 2xK - new candidtate keypoints
% S.F is 2xK - new canditate keypoints at first occurence
% S.T is 12xK - corresponding T at first occurence

run('param.m');

% Localization
KLT = vision.PointTracker('MaxBidirectionalError',KLT_max_bidirectional_error_cont, ...
    'BlockSize',KLT_patch_size_cont*ones(1,2),'MaxIterations',KLT_max_iterations_cont); % TODO: check out possible options 
initialize(KLT, fliplr(prev_S.P'), prev_I); 
[P_new,point_validity] = step(KLT,I);
S.X=prev_S.X(:, point_validity); % only keep the points that were tracked
S.P=flipud(P_new(point_validity,:)');

% plot all matches
if do_plotting
    subplot(2,4,[1,2]);  
    imshow(I); hold on;
    plot(S.P(2, :), S.P(1, :), 'rx', 'Linewidth', 2);
    plotMatches([1:size(S.P,2)], prev_S.P(:,point_validity), S.P, 2, 'g-');
    title('tracked keypoints')
end

% localize with P3P and ransac
[R_C_W, t_C_W, max_num_inliers_history, best_inlier_mask] = ransacP3P(S.X,flipud(S.P),K); % looks like ransacP3P nees switched x and y too

% only keep ransac inliers
if discard_p3p_outliers
    S.X = S.X(:,best_inlier_mask);
    S.P = S.P(:,best_inlier_mask);
end

% refine camera pose
[R_C_W, t_C_W] = nonlinearOptimization(S.X, S.P, R_C_W, t_C_W, K);

num_tracked_keypoints = size(S.P,2);
S.num_tracked_keypoints = num_tracked_keypoints;
fprintf('tracked keypoints: %i\n', num_tracked_keypoints);
T_C_W = [R_C_W, t_C_W];
T_C_W = T_C_W(:);

if isempty(R_C_W) || isempty(t_C_W)
    assert(false)
end


%% track keypoints
% find harris keypoints and patch descriptors
query_scores = harris(I, harris_patch_size_cont, harris_kappa_cont);
query_keypoints = selectKeypoints(query_scores, num_keypoints_cont, nonmaximum_supression_radius_cont);

if isempty(prev_S.C)
    S.C = query_keypoints;
    S.F = query_keypoints;
    S.T = repmat(T_C_W, 1, size(query_keypoints,2));
    S.num_added_keypoints = 0;

else
    % track triangulation canditate keypoints
    KLT_triang = vision.PointTracker('MaxBidirectionalError',KLT_max_bidirectional_error_cont, ...
    'BlockSize',KLT_patch_size_cont*ones(1,2),'MaxIterations',KLT_max_iterations_cont); % TODO: check out possible options 
    initialize(KLT_triang, fliplr(prev_S.C'), prev_I); 
    [C_new,p_validity] = step(KLT_triang,I);
    S.C = flipud(C_new(p_validity,:)');
    S.F = prev_S.F(:,p_validity);
    S.T = prev_S.T(:, p_validity); % only keep the points that were tracked
    
    % only triangulate points that come from the same image. otherwhise
    % this does not make sense
    unique_transforms = unique(S.T', 'rows')';
    
    if size(unique_transforms) > max_num_tracked_frames
        kill_old_transforms_mask = logical([zeros(1,max_num_tracked_frames), ones(size(unique_transforms,2)-max_num_tracked_frames)]);
    else
        kill_old_transforms_mask = logical(zeros(1, size(unique_transforms,2)));
    end
    for i=1:size(unique_transforms,2)
        transform_mask = all(S.T == unique_transforms(:,i),1);
        
        if sum(transform_mask) < min_points || kill_old_transforms_mask(i)
            S.C(:, transform_mask) = [];
            S.F(:, transform_mask) = [];
            S.T(:, transform_mask) = [];
            continue
        end
        
        T_C_W_i = reshape(unique_transforms(:,i),3,4);
        T_W_C_i = [T_C_W_i(1:3,1:3)', -T_C_W_i(1:3,1:3)'*T_C_W_i(1:3,4)];
        T_W_C = [R_C_W' -R_C_W'*t_C_W];
        %T_C1_C2 = T_C_W_i * T_W_C
         T_C1_C2 = [T_C_W_i(1:3,1:3) * T_W_C(1:3,1:3) T_C_W_i(1:3,1:3)*T_W_C(1:3,4)+T_C_W_i(1:3,4)];
         t_C1_C2 = T_C1_C2(1:3,4);
         
        %T_C2_C1 = T_C_W * T_W_C_i
        T_C2_C1 = [R_C_W * T_W_C_i(1:3,1:3), R_C_W*T_W_C_i(1:3,4) + t_C_W];
        
        database_keypoints = prev_S.F(:,p_validity);
        database_keypoints_homog = [flipud(database_keypoints(:,transform_mask)); ones(1,size(database_keypoints(:,transform_mask),2))];
        query_keypoints_homog = [flipud(S.C(:,transform_mask)); ones(1,size(S.C(:,transform_mask),2))];
        
        M_database = K * T_C_W_i;
        M_query = K * [R_C_W t_C_W];
        
        P_W = linearTriangulation(database_keypoints_homog, ...
           query_keypoints_homog, M_database, M_query);
        
       % drop points behind camera
        P_C1 = T_C_W_i * P_W;
        P_C2 = reshape(T_C_W, 3,4) * P_W;
        P_W = P_W(1:3,:);
        
        behind_camera_mask = bitor(P_C1(3, :) < 0, P_C2(3,:) < 0);
         
         P_C2_in_C1 = P_C1 - T_C1_C2(1:3,4);
         angles_deg = acosd(dot(P_C2_in_C1, P_C1) ./ (vecnorm(P_C2_in_C1,2) .* vecnorm(P_C1,2)));
         angles_mask = bitand(abs(angles_deg) > min_angle_deg, abs(angles_deg) < max_angle_deg);
         
         triangulate_mask = bitand(angles_mask, not(behind_camera_mask));
         
         % add new triangulated points
         new_points = P_W(:, triangulate_mask);
         %new_points_W = T_W_C_i*[new_points; ones(1,size(new_points,2))];
         S.X = [S.X new_points];
         matched_query_keypoints_i = S.C(:,transform_mask);
         S.P = [S.P matched_query_keypoints_i(:,triangulate_mask)];
         
         if do_plotting
            subplot(2,4,[1,2]);
            hold on
            new_P = matched_query_keypoints_i(:,triangulate_mask);
            plot(new_P(2,:), new_P(1, :), 'bo', 'Linewidth', 2);
            hold off
         end
            
         transform_cols = find(transform_mask);
         triangulated_cols = transform_cols(triangulate_mask);
         S.C(:, triangulated_cols) = [];
         S.F(:, triangulated_cols) = [];
         S.T(:, triangulated_cols) = [];      
    end
    
    % and new key points to F,C,T
    if ~isempty(T_C_W)
        
        % don't add if something close is already in C
        [~, mask_C] = nonMaxSuppression(query_keypoints, S.C, nonmaximum_supression_radius_cont, size(I)); 
        
        % don't add if something close is already in P
        [~, mask_P] = nonMaxSuppression(query_keypoints, S.P, nonmaximum_supression_radius_cont, size(I)); 
        
        new_keypoints = query_keypoints(:,bitand(mask_C,mask_P));
        S.C= [S.C new_keypoints];
        S.F =[S.F new_keypoints];
        S.T= [S.T repmat(T_C_W, [1, size(new_keypoints,2)])];
        assert(size(S.C,2) == size(S.T,2));
        
        fprintf('added new keypoints: %i', size(new_keypoints,2))
        S.num_added_keypoints = size(new_keypoints,2);
    end
    
    
    
end

end
