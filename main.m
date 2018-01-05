%%  Vision Algorithm for Mobile Robots
%   Visual odometry pipeline
%   Samuel Nyffenegger, Sebastian Ratz
%   Nov 2017 - Jan 2018

%% ToDo
%   cont tracking
%   cont supression
%   param tuning
%   new dataset

%% Setup
clear all; clc;
addpath(genpath(cd));

run('param.m');
%
path_to_main = fileparts(which('main.m')); 
if ds == 0
    kitti_path = [path_to_main,'/../data/kitti'];
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(2:end, [end-8 end]); % drop image0 to prevent index confusion
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    malaga_path = [path_to_main,'/../data/malaga-urban-dataset-extract-07'];
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    parking_path = [path_to_main,'/../data/parking'];
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    
elseif ds == 3
    alpha_omega_path = [path_to_main,'/../data/alpha_omega/'];
    run([alpha_omega_path,'Calib_Results.m']);
    K = [fc(1),0,cc(1); 0,fc(2),cc(2); 0,0,1];   

elseif ds == 4
    dubi_calib1_set1_path = [path_to_main,'/../data/Dubendorf/calib1/set1/'];
    run([dubi_calib1_set1_path,'../calibration/Calib_Results.m']);
    K = [fc(1),0,cc(1); 0,fc(2),cc(2); 0,0,1];   
    
elseif ds == 5
    duckietown_path_set1 = [path_to_main,'/../data/duckietown/set1/'];
    K = [361.58100437113353, 0.0, 317.9127184663754; ...
        0.0, 355.90465554417597, 243.37300302681604; ...
        0.0, 0.0, 1.0];
    last_frame = 3415;
elseif ds == 6
    duckietown_path_set2 = [path_to_main,'/../data/duckietown/set2/'];
    K = [361.58100437113353, 0.0, 317.9127184663754; ...
        0.0, 355.90465554417597, 243.37300302681604; ...
        0.0, 0.0, 1.0];
    last_frame = 2952;
else
    assert(false);
end

% Bootstrap
if ds == 0
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img2 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img2 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 3
    img1 = rgb2gray(imread([alpha_omega_path ...
        sprintf('/img_%02d.JPG',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([alpha_omega_path ...
        sprintf('/img_%02d.JPG',bootstrap_frames(2))]));    

elseif ds == 4
    img1 = rgb2gray(imread([dubi_calib1_set1_path ...
        sprintf('img%04d.JPG',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([dubi_calib1_set1_path ...
        sprintf('img%04d.JPG',bootstrap_frames(2))]));

elseif ds == 5
    img1 = rgb2gray(imread([duckietown_path_set1 ...
        sprintf('%05d.JPG',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([duckietown_path_set1 ...
        sprintf('%05d.JPG',bootstrap_frames(2))]));
elseif ds == 6
    img1 = rgb2gray(imread([duckietown_path_set2 ...
        sprintf('%05d.JPG',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([duckietown_path_set2 ...
        sprintf('%05d.JPG',bootstrap_frames(2))]));
            
else
    assert(false);
end


% %% initialization patch matching
% [inlier_query_keypoints, corresponding_landmarks, M_W_C2] = ...
%     initialization_patch_matching(img1, img2, K);
        
% initialization KLT
[inlier_query_keypoints, corresponding_landmarks, T_W_C2] = ...
  initialization_KLT(img1, img2, K);
T_W_C2

%% Continuous operation
run('param.m');

range = (bootstrap_frames(2)+1):last_frame;
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
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
        prev_image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i-1)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
        prev_image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i-1).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
        prev_image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i-1)])));
        
    elseif ds == 5
        image = rgb2gray(imread([duckietown_path_set1 ...
            sprintf('%05d.JPG',i)]));
        prev_image = rgb2gray(imread([duckietown_path_set1 ...
            sprintf('%05d.JPG',i-1)]));

    elseif ds == 6
        image = rgb2gray(imread([duckietown_path_set2 ...
            sprintf('%05d.JPG',i)]));
        prev_image = rgb2gray(imread([duckietown_path_set2 ...
            sprintf('%05d.JPG',i-1)]));

    else
        assert(false);
    end
   
    % do localization and triangulation
    [S, R_C_W, t_C_W] = processFrame(image,prev_image,prev_S, K);
    
    % fill the bags for post processing plots
    T_W_C = [R_C_W', -R_C_W'*t_C_W];
    if save_in_bags
        poses = [poses; T_W_C(:)'];
        landmarks = [landmarks, S.X];
        num_keypoints_statistics = [num_keypoints_statistics, [S.num_tracked_keypoints; S.num_added_keypoints]];
    else
        poses = [poses(end,:); T_W_C(:)'];
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
        if true
        old_X = intersect(prev_S.X', S.X', 'rows')';
        scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
        set(gcf, 'GraphicsSmoothing', 'on');
        view(0,0);
        axis equal;
        axis vis3d;
        axis(axis_array);
        end
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
        plot([i-1,i],[prev_S.num_tracked_keypoints,S.num_tracked_keypoints],'b.-'); 
        plot([i-1,i],[prev_S.num_added_keypoints,S.num_added_keypoints],'k.-'); 
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

%% post processing plots 2d
% ground truth (2d plot)
fig5 = figure(5); clf;  hold on; zoom on
    fig5.Position = full_screen;
    plot(poses(:,10),poses(:,12),'b-*','linewidth',3);
    axis equal; grid on; xlabel('x'); ylabel('z')
    plot(landmarks(1,:),landmarks(3,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    xlabel('x'); ylabel('z');

    
fig6 = figure(6); clf;  hold on; zoom on
    fig6.Position = full_screen;
    set(gca,'Ydir','reverse')
    plot(poses(:,10),poses(:,11),'b-*','linewidth',3);
    plot(landmarks(1,:),landmarks(2,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    axis equal; grid on; xlabel('x'); ylabel('y (watch direction)')


fig7 = figure(7); clf;  hold on; grid on; 
    fig7.Position = full_screen;
    plot([bootstrap_frames(2),range(1:size(num_keypoints_statistics,2)-1)],num_keypoints_statistics);
    xlabel('image frame number')
    legend('# tracked KPs','# added KPs','location','NE'); 

   
%% post processing plots 3d
fig8 = figure(8); clf;  hold on; 
    fig8.Position = full_screen;
    plot3(poses(:,10),poses(:,11),poses(:,12),'b-*','linewidth',3);
    axis equal; grid on; xlabel('x'); ylabel('z')
    plot(landmarks(1,:),landmarks(3,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    xlabel('x'); ylabel('z');

    
