%%  Vision Algorithm for Mobile Robots
%   Visual odometry pipeline
%   Samuel Nyffenegger, Sebastian Ratz
%   Nov 2017 - Jan 2018

%% Setup
clear all; clc;
addpath(genpath(cd));
run('param.m');

% load stuff
[img1, img2, K, last_frame, ground_truth, ds_path, left_images] = load_data_init();
        
% initialization KLT
[inlier_query_keypoints, corresponding_landmarks, T_W_C2] = ...
  initialization_KLT(img1, img2, K);


%% Continuous operation
run('param.m');

if early_stopping > 0
    warning('early stopping is activated.')
    range = (bootstrap_frames(2)+1):frame_step_size:(bootstrap_frames(2)+early_stopping);
else
    range = (bootstrap_frames(2)+1):frame_step_size:last_frame;
end

prev_S = struct('P',[],'X',[],'C',[],'F',[],'T',[]); 
prev_S.P = inlier_query_keypoints;
prev_S.X = corresponding_landmarks;
prev_S.num_tracked_keypoints = size(inlier_query_keypoints,2);
prev_S.num_added_keypoints = 0; 

if plot_tracking && do_plotting
    if plot_on_one_figure
        fig1 = figure(1); 
        fig1.Position = full_screen; 
        subplot(2,4,[3,4,7,8]);
    else
        figure(1); 
    end
    % plot initial landmarks
    scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis(axis_array);
    title('Trajectory of last 20 frames and landmarks (TODO: currently all)')
end

poses = T_W_C2(:)'; 
landmarks = corresponding_landmarks; 
num_keypoints_statistics = [size(corresponding_landmarks,2);0];
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    
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
    
    % plot
    if plot_landmarks && do_plotting
        if plot_on_one_figure
            fig1 = figure(1); 
            fig1.Position = full_screen; 
            subplot(2,4,[3,4,7,8]);
        else
            figure(2); 
        end

        hold on; grid on; 
        if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
            plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
            view(0,0);
        end
         new_X = setdiff(S.X', prev_S.X', 'rows')';
        if ~isempty(new_X)
            scatter3(new_X(1, :), new_X(2, :), new_X(3, :), 5, 'r');
        end
        old_X = intersect(prev_S.X', S.X', 'rows')';
        scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
        set(gcf, 'GraphicsSmoothing', 'on');
        view(0,0);
        axis equal;
        axis vis3d;
        axis(axis_array);
        xlabel('x'); ylabel('y'); zlabel('z');
        hold off
    end
    
    
    if plot_on_one_figure && do_plotting
    
        if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
            figure(1); subplot(2,4,6); hold on; grid on; axis equal;      
                plot([poses(end-1,10),poses(end,10)],[poses(end-1,12),poses(end,12)],'b.-')
                title('full trajectory'); axis equal; grid on;
                xlabel('x'); ylabel('z');
        end

        figure(1);  subplot(2,4,5); hold on; grid on; 
            plot([i-frame_step_size,i],[prev_S.num_tracked_keypoints,S.num_tracked_keypoints],'b.-'); 
            plot([i-frame_step_size,i],[prev_S.num_added_keypoints,S.num_added_keypoints],'k.-'); 
            legend('# tracked KPs', '# added KPs','location','NE');
            xlabel('iteration');
            title('keypoints statistics')

    end 
    
    if isempty(t_C_W)
        sprintf('translation is empty. failed to localize.')
    end

    % Makes sure that plots refresh.    
    % pause(0.01);

    prev_img = image;
    prev_S = S;
end
fprintf('\ncongratulation!\n')

    
