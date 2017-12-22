function [R_C_W, t_C_W, max_num_inliers_history] = ransacP3P(X,P,K)
best_R = [];
best_t = [];
estimate_DLT = false;
tweaked_for_more = true;
best_error_sum = -1;
% parameters
if tweaked_for_more
    num_iterations = 1000;
else
    num_iterations = 200;
end
pixel_tolerance = 10;
k = 3;
% RANSAC
first_time = true;
max_num_inliers = 0;
max_num_inliers_history = zeros(1, num_iterations);
for i = 1:num_iterations
    % Model from k samples (3P)
    [X_sample, idx] = datasample(...
        X, k, 2, 'Replace', false);
    P_sample = P(:, idx);

    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = K\[P_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end

    poses = p3p(X_sample, normalized_bearings);

    % Decode p3p output
    R_C_W_guess = zeros(3, 3, 4);
    t_C_W_guess = zeros(3, 1, 4);
    for ii = 0:3        
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end

    if tweaked_for_more
        min_inlier_count = 30;
    else
        min_inlier_count = 6;
    end
    % Count inliers:
    for ii=1:4
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,ii) * X) + ...
            repmat(t_C_W_guess(:,:,ii), ...
            [1 size(X, 2)]), K);
        difference = P - projected_points;
        errors = sum(difference.^2, 1);
        is_inlier = errors < pixel_tolerance^2;

        if nnz(is_inlier) > max_num_inliers && ...
             nnz(is_inlier) >= min_inlier_count
            max_num_inliers = nnz(is_inlier);        
            best_inlier_mask = is_inlier;
            best_R = R_C_W_guess(:,:,ii);
            best_t = t_C_W_guess(:,:,ii);
        end
    end
    max_num_inliers_history(i) = max_num_inliers;
    
end
if estimate_DLT
    if max_num_inliers == 0
        R_C_W = [];
        t_C_W = [];
    else
        M_C_W = estimatePoseDLT(...
            P(:, best_inlier_mask>0)', ...
            X(:, best_inlier_mask>0)', K);
        R_C_W = M_C_W(:, 1:3);
        t_C_W = M_C_W(:, end);
    end
else % estimate with P3P
    R_C_W = best_R;
    t_C_W = best_t;
    
end 

end