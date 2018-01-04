function [S, R_C_W, t_C_W] = processFrame(I,prev_I,prev_S,K)
% Params:
% S.P is 2xN - keypoints
% S.X is 3xN - 3D landmarks
% S.C is 2xK - new candidtate keypoints
% S.F is 2xK - new canditate keypoints at first occurence
% S.T is 12xK - corresponding T at first occurence

run('param.m');

% Localization
KLT = vision.PointTracker('MaxBidirectionalError', 1); % TODO: check out possible options 
initialize(KLT, fliplr(prev_S.P'), prev_I); 
[P_new,point_validity] = step(KLT,I);
S.X=prev_S.X(:, point_validity); % only keep the points that were tracked
S.P=flipud(P_new(point_validity,:)');

% plot all matches
if plot_tracking 
    figure(2);
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

else
    % track triangulation canditate keypoints
    KLT_triang = vision.PointTracker('MaxBidirectionalError', 1); % TODO: check out possible options 
    initialize(KLT_triang, fliplr(prev_S.C'), prev_I); 
    [C_new,p_validity] = step(KLT_triang,I);
    S.C= flipud(C_new(p_validity,:)');
    S.F = prev_S.F(:,p_validity);
    S.T=prev_S.T(:, p_validity); % only keep the points that were tracked
    
    if plot_tracking 
    figure(3);
    title('matches new keypoints')
    imshow(I); hold on;
    plot(S.C(2, :), S.C(1, :), 'rx', 'Linewidth', 2);

    plotMatches([1:size(prev_S.C(:,p_validity),2)], prev_S.C(:,p_validity), S.C, 2, 'g-');
    pause(0.001);
    hold off
    end
    
    % only triangulate points that come from the same image. otherwhise
    % this does not make sense
    unique_transforms = unique(S.T', 'rows')';
    size(unique_transforms)
    for i=1:size(unique_transforms,2)
        transform_mask = all(S.T == unique_transforms(:,i),1);
        
        if sum(transform_mask) < min_points
            S.C(:, transform_mask) = [];
            S.F(:, transform_mask) = [];
            S.T(:, transform_mask) = [];
            continue
        end
        
        [R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
        max_num_inliers_history] = estimateProjectionRANSAC(S.F(:,transform_mask), ...
        S.C(:,transform_mask), K, n_iterations_triang, pixel_tolerance);

        % rescale translation
        T_C_W_i = reshape(unique_transforms(:,i), 3,4);
        t_C_W_i = T_C_W_i(1:3,4);
        
        T_W_C_i = [T_C_W_i(1:3,1:3)' -T_C_W_i(1:3,1:3)'*T_C_W_i(1:3,4)];
        
        t_C2_C1_localization = t_C_W - t_C_W_i;
        norm_t_localization = norm(t_C2_C1_localization);
        norm_t_triangulation = norm(t_C2_C1);
        
        scale_correction = norm_t_localization / norm_t_triangulation;
        
        t_C2_C1 = scale_correction * t_C2_C1;
        P_C2 = P_C2 * scale_correction;
        
        if isempty(t_C2_C1) || isempty(R_C2_C1)
            continue
        end
        figure(18);
        subplot(3,1,1); 
        hold on
        title('tC2C1')
        quiver3(0,0,0, t_C2_C1(1), t_C2_C1(2), t_C2_C1(3))
        axis([-5 5 -1 1 -1 1]);
        view(0,0);
        hold off
        figure(18);
        subplot(3,1,2);
        hold on
        title('tC2C1 localization')
        quiver3(0,0,0, t_C2_C1_localization(1), t_C2_C1_localization(2), t_C2_C1_localization(3))
        axis([-5 5 -1 1 -1 1]);
        view(0,0); 
        hold off
        subplot(3,1,3);
        hold on
        title('tCW_i ')
        quiver3(0,0,0, t_C_W_i(1), t_C_W_i(2), t_C_W_i(3))
        axis([-5 5 -1 1 -1 1]);
        view(0,0); 
        hold off
        
        % drop points behind camera
        behind_camera_mask = P_C2(3, :) < 0;
        
         % triangulated points in original coordinate frame 
         P_C1 = R_C2_C1' * P_C2 - R_C2_C1' * t_C2_C1;
         P_C1_in_C2 = P_C2 - t_C2_C1; % points from C1 to P in frame C2.

         angles_deg = acosd(dot(P_C1_in_C2, P_C2) ./ (vecnorm(P_C1_in_C2,2) .* vecnorm(P_C2,2)));
         angles_mask = abs(angles_deg) > alpha_deg;
         
         if sum(angles_mask)/size(angles_mask,2) > 0.5
            triangulate_mask = bitand(angles_mask, not(behind_camera_mask));
         else
             triangulate_mask = logical(zeros(1,size(angles_mask,2)));
         end
         
         % add new triangulated points
         new_points = P_C1(:, triangulate_mask);
         new_points_W = T_W_C_i*[new_points; ones(1,size(new_points,2))];
         S.X = [S.X new_points_W(1:3,:)];
         matched_query_keypoints_i = S.C(:,transform_mask);
         S.P = [S.P matched_query_keypoints_i(:,triangulate_mask)];
         
         if plot_tracking
            figure(2);
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
        S.T= [S.T repmat(T_C_W(:), [1, size(new_keypoints,2)])];
        assert(size(S.C,2) == size(S.T,2));
        
        sprintf('added new keypoints: %i', size(new_keypoints,2))
    end
    
end

end
