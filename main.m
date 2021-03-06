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


% Continuous operation
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
        
    subplot(2,4,6); hold on; grid on; axis equal;  
        t_W_C2 = T_W_C2(1:3,4);
        plot([0,t_W_C2(1)],[0,t_W_C2(3)],'b.-');
        plot([0,t_W_C2(1)],[0,t_W_C2(3)],'g.-')
        plot([0,t_W_C2(1)],[0,t_W_C2(3)],'b.-');
end

% prepare to run loop
poses = T_W_C2(:)'; 
landmarks = corresponding_landmarks; 
num_keypoints_statistics = [size(corresponding_landmarks,2);0];

% BA variables
states_BA = [];
poses_BA=zeros(12,window_size);
states_BA_boundary = [];
poses_BA_boundary = [];
index_shift = mod(range(1),window_size*frame_step_size)+boundary_window_size*frame_step_size;
states_refined_all = [];
T_W_Cs_all = [];

for j=1:sliding_window_plots_number; landmarks_container{j} = []; end

for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    updated_BA = false;
    
    % load images
    [prev_image, image] = load_data_cont(i, ds_path, left_images);
   
    % do localization and triangulation
    tic 
    [S, R_C_W, t_C_W] = processFrame(image,prev_image,prev_S, K);
    toc
    T_W_C = [R_C_W', -R_C_W'*t_C_W];
    
    if isempty(t_C_W)
        sprintf('\ntranslation is empty. failed to localize.')
        assert(false)
    end

    if do_bundle_adjustment
        % first frames are not optimized but serve as boundary condition
        if i < range(1)+boundary_window_size*frame_step_size
            frame_number = -1;
            states_BA_boundary = [states_BA_boundary S];
            poses_BA_boundary = [poses_BA_boundary T_W_C(:)];
        else        
            frame_number = mod(i-index_shift,window_size*frame_step_size)/frame_step_size+1; % 1 <= frame_number <= window size
            states_BA = [states_BA S];
            poses_BA(:,frame_number) = T_W_C(:);
        end

        % bundle adjustment after end of window
        if mod(frame_number,window_size) == 0
            fprintf('executing bundle adjustment\n')
            tic
            %convert data format of states to be optimized  
            [hidden_state, observations, poses_boundary, all_landmarks, state_landmark_ids, num_boundary_observations] =...
                getBAFormat(states_BA, states_BA_boundary, poses_BA, poses_BA_boundary);

            % execute BA
            hidden_state_opt = runBAWindow(hidden_state,observations, poses_boundary, all_landmarks, state_landmark_ids, num_boundary_observations, K, max_iters);

            % convert back to standard representation
            [T_W_Cs, states_refined] = getStandardFormat(hidden_state_opt, observations,all_landmarks,state_landmark_ids, states_BA, window_size);
            S = states_refined(end); % set optimized frame as current frame
            T_W_C = reshape(T_W_Cs(:,end),3,4); % overwrite last pose with refined one
            R_C_W = T_W_C(1:3,1:3)';
            t_C_W = -T_W_C(1:3,1:3)'*T_W_C(1:3,4);

            states_refined_all = [states_refined_all states_refined];
            T_W_Cs_all = [T_W_Cs_all T_W_Cs];

            % reset collectors for BA
            states_BA_boundary = states_refined(end-boundary_window_size+1:end);
            poses_BA_boundary = T_W_Cs(:,end-boundary_window_size+1:end);
            states_BA = [];
            poses_BA=zeros(12,window_size);
            updated_BA=true;
            toc
        end
    end
    
    % fill the bags for post processing plots
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
        fprintf('plotting\n')
        tic
        if updated_BA
            plotOverview_cont(S, prev_S, R_C_W, t_C_W, poses, i, landmarks_container, T_W_Cs_all);
        else
            plotOverview_cont(S, prev_S, R_C_W, t_C_W, poses, i, landmarks_container, []);
        end
        toc
    end
    
    % prepare for next iteration
    prev_img = image;
    prev_S = S;    
     
    pause(0.001);

end
fprintf('\ncongratulation!\n')
