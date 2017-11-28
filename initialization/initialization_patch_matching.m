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
num_keypoints = 1000;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda =6;


% bridge
try
    % launched inside initialization_patch_matching
    query_image = img2;
    database_image = img1;
catch
    % launched outside
end


% find keypoints and descriptors
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

% use normalized 8-point algorithm in RANSAC fashion to obtain fundamental matrix
[F, inliers_mask, status] = estimateFundamentalMatrix(matched_query_keypoints',matched_database_keypoints','Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);
inliers_mask = inliers_mask';
if status ~= 0
    warning(['estimateFundamentalMatrix did not find enough inlier pairs, only ',num2str(nnz(inliers_mask))])
end

% indexing to get matched inliers
n_matched_inlier_keypoints = nnz(inliers_mask)
inliers_matches = all_matches(all_matches>0).*inliers_mask;
all_inliers_mask = all_matches > 0;
all_inliers_mask(all_inliers_mask>0) = inliers_matches;
all_inliers_matches = all_matches.*all_inliers_mask;

if plot_all_matches 
    figure(1); clf; 
        imshow(query_image); hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatches(all_matches, query_keypoints, database_keypoints, 2, 'g-')
        if plot_inlier_matches
            plotMatches(all_inliers_matches, query_keypoints, database_keypoints, 3, 'b-')
        end
end
%%
% pi are the homogeneous mateched {query,database} keypoints
p1 = [matched_query_keypoints;ones(1,length(matched_query_keypoints))];
p2 = [matched_database_keypoints;ones(1,length(matched_database_keypoints))];

% essential matrix, rotation matrix and translation vector computation
E = K'*F*K; 
[Rots,u3] = decomposeEssentialMatrix(E);
[R_C2_C1,T_C2_C1] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);

% Triangulate a point cloud using the final transformation (R,T)
M1 = K * eye(3,4);
M2 = K * [R_C2_C1, T_C2_C1];
matched_landmarks = linearTriangulation(p1,p2,M1,M2);
matched_landmarks = matched_landmarks(1:3,:);

% assignement using inliers_index (TODO: faster if only those landmarks 
% are calculated)
inlier_query_keypoints = matched_query_keypoints; %(:,inliers_index);
corresponding_landmarks = matched_landmarks; %(:,inliers_index);


%% plot
% figure(1)
% subplot(1,3,1)
% 
% % R,T should encode the pose of camera 2, such that M1 = [I|0] and M2=[R|t]
% 
% % P is a [4xN] matrix containing the triangulated point cloud (in
% % homogeneous coordinates), given by the function linearTriangulation
% plot3(matched_landmarks(1,:), matched_landmarks(2,:), matched_landmarks(3,:), 'o');
% 
% % Display camera pose
% 
% plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
% text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');
% 
% center_cam2_W = -R_C2_C1'*T_C2_C1;
% plotCoordinateFrame(R_C2_C1',center_cam2_W, 0.8);
% text(center_cam2_W(1)-0.1, center_cam2_W(2)-0.1, center_cam2_W(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');
% 
% axis equal
% rotate3d on;
% grid
% 
% % Display matched points
% subplot(1,3,2)
% imshow(query_image,[]);
% hold on
% plot(p1(1,:), p1(2,:), 'ys');
% title('Image 1')
% 
% subplot(1,3,3)
% imshow(database_image,[]);
% hold on
% plot(p2(1,:), p2(2,:), 'ys');
% title('Image 2')


end



