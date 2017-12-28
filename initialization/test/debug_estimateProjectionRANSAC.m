clear all; clc;

load('workspace_crash.mat');
[R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, ...
        max_num_inliers_history] = estimateProjectionRANSAC(matched_database_keypoints(:,transform_mask), ...
        matched_query_keypoints(:,transform_mask), K, n_iterations, pixel_tolerance);
    
size(P_C2)
size(best_inlier_mask)



