function [img1, img2, K, last_frame, ground_truth, ds_path, left_images] = load_data_init()
% loads images for corresponding dataset

    run('param.m');

    path_to_main = fileparts(which('main.m')); 
    if ds == 0
        ds_path = [path_to_main,'/../data/kitti'];
        ground_truth = load([ds_path '/poses/00.txt']);
        ground_truth = ground_truth(2:end, [end-8 end]); % drop image0 to prevent index confusion
        last_frame = 4540;
        K = [7.188560000000e+02 0 6.071928000000e+02
            0 7.188560000000e+02 1.852157000000e+02
            0 0 1];
    elseif ds == 1
        ds_path = [path_to_main,'/../data/malaga-urban-dataset-extract-07'];
        images = dir([ds_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
        left_images = images(3:2:end);
        last_frame = length(left_images);
        K = [621.18428 0 404.0076
            0 621.18428 309.05989
            0 0 1];
    elseif ds == 2
        ds_path = [path_to_main,'/../data/parking'];
        last_frame = 598;
        K = load([ds_path '/K.txt']);
        ground_truth = load([ds_path '/poses.txt']);
        ground_truth = ground_truth(:, [end-8 end]);

    % elseif ds == 3
    %     alpha_omega_path = [path_to_main,'/../data/alpha_omega/'];
    %     run([alpha_omega_path,'Calib_Results.m']);
    %     K = [fc(1),0,cc(1); 0,fc(2),cc(2); 0,0,1];   
    % 
    % elseif ds == 4
    %     dubi_calib1_set1_path = [path_to_main,'/../data/Dubendorf/calib1/set1/'];
    %     run([dubi_calib1_set1_path,'../calibration/Calib_Results.m']);
    %     K = [fc(1),0,cc(1); 0,fc(2),cc(2); 0,0,1];   
    elseif ds == 5
        ds_path = [path_to_main,'/../data/duckietown/set1/'];
        K = [361.58100437113353, 0.0, 317.9127184663754; ...
            0.0, 355.90465554417597, 243.37300302681604; ...
            0.0, 0.0, 1.0];
        last_frame = 3415;
    elseif ds == 6
        ds_path = [path_to_main,'/../data/duckietown/set2/'];
        K = [361.58100437113353, 0.0, 317.9127184663754; ...
            0.0, 355.90465554417597, 243.37300302681604; ...
            0.0, 0.0, 1.0];
        last_frame = 2952;
    else
        assert(false);
    end

    % Bootstrap
    if ds == 0
        img1 = imread([ds_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(1))]);
        img2 = imread([ds_path '/00/image_0/' ...
            sprintf('%06d.png',bootstrap_frames(2))]);
    elseif ds == 1
        img1 = rgb2gray(imread([ds_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(1)).name]));
        img2 = rgb2gray(imread([ds_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(bootstrap_frames(2)).name]));
    elseif ds == 2
        img1 = rgb2gray(imread([ds_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
        img2 = rgb2gray(imread([ds_path ...
            sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

    % elseif ds == 3
    %     img1 = rgb2gray(imread([alpha_omega_path ...
    %         sprintf('/img_%02d.JPG',bootstrap_frames(1))]));
    %     img2 = rgb2gray(imread([alpha_omega_path ...
    %         sprintf('/img_%02d.JPG',bootstrap_frames(2))]));    
    % 
    % elseif ds == 4
    %     img1 = rgb2gray(imread([dubi_calib1_set1_path ...
    %         sprintf('img%04d.JPG',bootstrap_frames(1))]));
    %     img2 = rgb2gray(imread([dubi_calib1_set1_path ...
    %         sprintf('img%04d.JPG',bootstrap_frames(2))]));
    elseif ds == 5
        img1 = rgb2gray(imread([ds_path ...
            sprintf('%05d.JPG',bootstrap_frames(1))]));
        img2 = rgb2gray(imread([ds_path ...
            sprintf('%05d.JPG',bootstrap_frames(2))]));
    elseif ds == 6
        img1 = rgb2gray(imread([ds_path ...
            sprintf('%05d.JPG',bootstrap_frames(1))]));
        img2 = rgb2gray(imread([ds_path ...
            sprintf('%05d.JPG',bootstrap_frames(2))]));        
    else
        assert(false);
    end
    
    
    % dummy assignement (malaga, duckietown)
    if ~exist('ground_truth','var'); ground_truth = []; end
    if ~exist('left_images','var'); left_images = []; end
    
    
end