%%  Vision Algorithm for Mobile Robots
%   Visual odometry pipeline
%   Samuel Nyffenegger, Sebastian Ratz
%   Nov 2017 - Jan 2018

%% Setup
clear all; close all; clc;
addpath(genpath(cd));

ds = 0; % 0: KITTI, 1: Malaga, 2: parking

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
    bootstrap_frames = [1,3];
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img2 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    bootstrap_frames = []; % tba
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img2 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    bootstrap_frames = []; % tba
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img2 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

% initialization 
[inlier_query_keypoints, corresponding_landmarks] = ...
    initialization_patch_matching(img1, img2, K);

%% plot ground truth
plot_animated = false; 

idx = 1:3; %size(ground_truth,1); 
if plot_animated
    figure(2); clf; hold on; 
        axis([-300,300,-100,500]); grid on;
        xlabel('x'); ylabel('y'); title('ground truth')
        for i = idx(1:end-1)
            plot(ground_truth(i:i+1,1),ground_truth(i:i+1,2),'k-');
            pause(0.001);
        end
else
    figure(1); clf; hold on; grid on;
        plot(ground_truth(idx,1),ground_truth(idx,2),'k-');    
        xlabel('x'); ylabel('y'); title('ground truth');
        axis equal
end

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end
