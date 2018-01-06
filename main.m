%%  Vision Algorithm for Mobile Robots
%   Visual odometry pipeline
%   Samuel Nyffenegger, Sebastian Ratz
%   Nov 2017 - Jan 2018

%% Initialization
clear all; clc;
addpath(genpath(cd));
run('param.m');

% load stuff
[img1, img2, K, last_frame, ground_truth, ds_path, left_images] = load_data_init();
        
% initialization KLT
[inlier_query_keypoints, corresponding_landmarks, T_W_C2] = initialization_KLT(img1, img2, K);


%% Continuous operation
run('param.m');

% save stuff from initialization
range = (bootstrap_frames(2)+1):frame_step_size:last_frame;
prev_S = struct('P',[],'X',[],'C',[],'F',[],'T',[]); 
prev_S.P = inlier_query_keypoints;
prev_S.X = corresponding_landmarks;
prev_S.num_tracked_keypoints = size(inlier_query_keypoints,2);
prev_S.num_added_keypoints = 0; 

% plot initial landarks
if do_plotting
    fig1 = figure(1); clf; % delete initialization figure, other format 
    fig1.Position = full_screen; 
    subplot(2,4,[3,4,7,8]);
        scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
        set(gcf, 'GraphicsSmoothing', 'on'); view(0,0);
        axis equal; axis vis3d; axis(axis_array);
        xlabel('x'); ylabel('y'); zlabel('z');
        title('Trajectory of last 20 frames and landmarks (TODO: currently all)')
end

% prepare to run loop
poses = T_W_C2(:)'; 
landmarks = corresponding_landmarks; 
num_keypoints_statistics = [size(corresponding_landmarks,2);0];
for j=1:sliding_window_plots_number; landmarks_container{j} = []; end
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    % if sliding_window_plots; figure(1); clf; end
    
    % load images
    [prev_image, image] = load_data_cont(i, ds_path, left_images);
   
    % do localization and triangulation
    [S, R_C_W, t_C_W] = processFrame(image,prev_image,prev_S, K);
        
    % fill the bags for post processing plots
    T_W_C = [R_C_W', -R_C_W'*t_C_W];
    if save_in_bags
        poses = [poses; T_W_C(:)'];
        landmarks = [landmarks, S.X];
        num_keypoints_statistics = [num_keypoints_statistics, [S.num_tracked_keypoints; S.num_added_keypoints]];
    else
        poses = [poses(max(1,end-sliding_window_plots_number+2):end,:); T_W_C(:)'];
    end
    
    if sliding_window_plots
        for j = sliding_window_plots_number:-1:2
            landmarks_container{j} = landmarks_container{j-1};
        end       
        landmarks_container{1} = prev_S.X;
    else
        landmarks_container = [];
    end
    
    % plot stuff
    if do_plotting
        plotOverview_cont(S, prev_S, R_C_W, t_C_W, poses, i, landmarks_container);
    end
    
    % prepare for next iteration
    prev_img = image;
    prev_S = S;
     
    pause(0.001);
end
fprintf('\ncongratulation!\n')
