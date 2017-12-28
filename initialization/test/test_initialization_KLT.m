%% simple setup to test KLT 
% convention:
% keypoints 2xN = [y1, y2, ...; x1, x2,...]
% points used for KLT tracking: Nx2 = [x1, y2; x2, y2; ...]

clear all; clc;
run('param.m');

% load images with stars
database_image = rgb2gray(imread('star1.png'));
query_image = rgb2gray(imread('star2.png'));
[H, W] = size(database_image);
database_image = database_image(70:H-70, 110:W-110);
query_image = query_image(70:H-70, 110:W-110);
[H, W] = size(database_image);


% initialize
num_keypoints = 5;
% nonmaximum_supression_radius = 8;

% get harris keypoints on first (database) image
database_scores = harris(database_image, harris_patch_size, harris_kappa);
database_keypoints = selectKeypoints(database_scores, num_keypoints, nonmaximum_supression_radius);
database_descriptors = describeKeypoints(database_image, database_keypoints, descriptor_radius);

% track keypoints using Lucas Kanade tracking (KLT)
patch_size = 2*harris_patch_size+1;
tracker = vision.PointTracker('MaxBidirectionalError',1,'BlockSize',patch_size*ones(1,2)); 
initialize(tracker,fliplr(database_keypoints'),database_image);
[query_keypoints, point_validity] = step(tracker,query_image);

printmat([fliplr(database_keypoints'),query_keypoints,point_validity],'keypoints','#1 #2 #3 #4 #5','db_x db_y qr_x qr_y validity')
query_keypoints = fliplr(query_keypoints)';

% plot all matches
plot_tracking = true;
if plot_tracking 
    figure(1); clf; 
        imshow(query_image); hold on;  
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatches([1:size(query_keypoints(:,point_validity),2)], database_keypoints(:,point_validity), query_keypoints(:,point_validity), 2, 'g-');
end