%%  Vision Algorithm for Mobile Robots
%   Visual odometry pipeline
%   Samuel Nyffenegger, Sebastian Ratz
%   Nov 2017 - Jan 2018

%% Setup
clear all; clc;
addpath(genpath(cd));
run('param.m');

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
else
    assert(false);
end

%% initialization 
[inlier_query_keypoints, corresponding_landmarks, M_W_C2] = ...
    initialization_patch_matching(img1, img2, K);
        
%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
prev_S = struct('P',[],'X',[],'C',[],'F',[],'T',[])
prev_S.P = flipud(inlier_query_keypoints);
prev_S.X = corresponding_landmarks;
    figure(2);
     scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis([-40 10 -10 5 -1 60]);
for i = 46:75%range
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
    else
        assert(false);
    end
   
    % do localization and triangulation
    [S, R_C_W, t_C_W] = processFrame(image,prev_image,prev_S, K);
    
    % plot
    plot = true;
    if plot
        hold on
        if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
            new_X = setdiff(S.X', prev_S.X', 'rows')'
            if ~isempty(new_X)
                scatter3(new_X(1, :), new_X(2, :), new_X(3, :), 5, 'r');
            end
            plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
            view(0,0);
        end
    end
    t_C_W
    hold off


    % Makes sure that plots refresh.    
    pause(0.01);

    prev_img = image;
    prev_S = S
end
