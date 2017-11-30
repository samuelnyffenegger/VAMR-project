%% Test estimateProjectionRANSAC
clear all; close all; clc;
path_to_file = fileparts(which('test_estimateFundamentalMatrix.m')); 
path_to_code = [path_to_file,'/../..'];
addpath(genpath(path_to_code));

% load images
database_image_colour = imread('initialization/test/fountain_1.jpg');
query_image_colour = imread('initialization/test/fountain_2.jpg');
database_image = rgb2gray(database_image_colour);
query_image = rgb2gray(query_image_colour);

% control parameter
plot_all_matches = true;

% parameters
K = [1379.74 0 760.35
    0 1382.08 503.41
    0 0 1 ];
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 400;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 7;

% prepare points using given (outlier-free) correspondances
p1 = load('matches0001.txt');
p2 = load('matches0002.txt');
p1 = [p1;ones(1,length(p1))];
p2 = [p2;ones(1,length(p2))];
E = estimateEssentialMatrix(p1, p2, K, K);
[Rots,u3] = decomposeEssentialMatrix(E);
[R_C2_W,T_C2_W] = disambiguateRelativePose(Rots,u3,p1,p2,K,K);
M1 = K * eye(3,4);
M2 = K * [R_C2_W, T_C2_W];
P = linearTriangulation(p1,p2,M1,M2);

% find keypoints and descriptors (there will be outliers)
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
        imshow(query_image_colour); hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatches(all_matches, query_keypoints, database_keypoints, 2, 'g-')
end

%% compute essential matrix in RANSAC fashion

[R_C2_C1, t_C2_C1, P_C2, best_inlier_mask, max_num_inliers_history] = ...
    estimateProjectionRANSAC(matched_database_keypoints, matched_query_keypoints, K);

M_C2_C1 = [R_C2_C1, t_C2_C1]

% convert point cloud in C2 frame into C1 (W) frame
P_C1 = (R_C2_C1' * P_C2) + repmat(-R_C2_C1'*t_C2_C1,[1 size(P_C2, 2)]); 

% Visualize the 3-D scene
center_myCam2_C1 = -R_C2_C1'*t_C2_C1;
center_cam2_C1 = -R_C2_W'*T_C2_W;

figure(2),
    subplot(4,1,1:3)
        plot3(P(1,:), P(2,:), P(3,:), 'bo');

        plotCoordinateFrame(eye(3),zeros(3,1), 0.8);
        text(-0.1,-0.1,-0.1,'Cam 1','fontsize',10,'color','k','FontWeight','bold');

        plotCoordinateFrame(R_C2_C1',center_myCam2_C1, 0.8);
        text(center_myCam2_C1(1)-0.1, center_myCam2_C1(2)-0.1, center_myCam2_C1(3)-0.1,'myCam 2','fontsize',10,'color','k','FontWeight','bold');

        plotCoordinateFrame(R_C2_W',center_cam2_C1, 0.8);
        text(center_cam2_C1(1)-0.1, center_cam2_C1(2)-0.1, center_cam2_C1(3)-0.1,'Cam 2','fontsize',10,'color','k','FontWeight','bold');

        xlabel('x'); ylabel('y'), zlabel('z');
        axis equal
        axis([-2.5,2.5,-1,1,-0.8 6])
        rotate3d on;
        grid
        view([0,0])
        title('3d scene with camera frames')
    
    subplot(414)
        plot(max_num_inliers_history)
        title('Maximum inlier count over RANSAC iterations.');

% camera pose error analysis
SSD_camera_pose = sum(abs(center_cam2_C1-center_myCam2_C1).^2)



