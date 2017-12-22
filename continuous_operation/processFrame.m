function [S, R_C_W, t_C_W] = processFrame(I,prev_I,prev_S,K)
% Params:
% S.P is 2xN - keypoints
% S.X is 3xN - 3D landmarks
% S.C is 2xK - new candidtate keypoints
% S.F is patch_sizexK - corresponding descriptors at first occurence
% S.T is 12xK - corresponding T at first occurence

% Localization
KLT = vision.PointTracker('BlockSize',[501,501], 'MaxIterations', 60) % TODO: check out possible options
initialize(KLT, prev_S.P', prev_I');
[P_new,point_validity] = step(KLT,I');
S.X=prev_S.X(:, point_validity); % only keep the points that were tracked
S.P=P_new(point_validity,:)';

% plot all matches
plot_tracking = false;
if plot_tracking 
    figure; clf; 
        imshow(I); hold on;
        plot(prev_S.P(2, :), prev_S.P(1, :), 'rx', 'Linewidth', 2);
        plotMatches([1:size(S.P,2)], prev_S.P(:,point_validity), S.P, 2, 'g-')
        pause(0.001);
end



[R_C_W, t_C_W, max_num_inliers_history] = ransacP3P(S.X,S.P,K);
max_num_inliers_history(end)
T_C_W = [R_C_W, t_C_W];
T_C_W = T_C_W(:);





%% track keypoints in C
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

if size(database_keypoints,2) > 0
    % match descriptors 
    all_matches = matchDescriptors(query_descriptors, database_descriptors, match_lambda);
    matched_query_mask = all_matches > 0;
    matched_query_keypoints = query_keypoints(:,matched_query_mask);
    matched_database_index = all_matches(all_matches > 0);
    matched_database_keypoints = database_keypoints(:,matched_database_index);
    assert(size(matched_database_keypoints,2) == size(matched_query_keypoints,2));
    n_matched_keypoints = size(matched_database_keypoints,2);

    matched_database_descriptors = database_descriptors(:,matched_database_index);
    matched_database_transforms = prev_S.T(:,matched_database_index);
    
    if all(size(T_C_W) > 0)
        unmatched_query_keypoints = query_keypoints(:,not(matched_query_mask));
        unmatched_query_descriptors = query_descriptors(:,not(matched_query_mask));
        unmatched_query_transforms = repmat(T_C_W, 1, size(unmatched_query_keypoints,2));
        % add matched database keypoints and newly found keypoints
        S.C=[matched_database_keypoints unmatched_query_keypoints];
        S.F=[matched_database_descriptors unmatched_query_descriptors];
        S.T=[matched_database_transforms unmatched_query_transforms];
        assert(size(S.C,2) == size(S.F,2));
        assert(size(S.F,2) == size(S.T,2));
    else
        S.C=[matched_database_keypoints];
        S.F=[matched_database_descriptors];
        S.T=[matched_database_transforms];
    end
else
    S.C = query_keypoints;
    S.F = query_descriptors;
    S.T = repmat(T_C_W, 1, size(query_keypoints,2));
end

%% Triangulate new points



end
