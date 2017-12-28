%% control parameters
talkative_initialization = true;

%% tuning parameters
% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 400;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 10;

% patch matching with RANSAC
n_iterations_matching_RANSAC = 1000;
pixel_tolerance_RANSAC = 3; 

%% dataset specific tuning parameters
ds = 0;         % dataset - 0: KITTI, 1: Malaga, 2: parking
switch ds
    case 0 % Kitti parameters
        bootstrap_frames = [0,2]; 
        % bootstrap_frames = [100,102];

    case 1 % Malaga parameters
        bootstrap_frames = []; % tba

    case 2 % parking parameters
        bootstrap_frames = [40,45]; 
    
    otherwise
        warning('choose dataset!')
end


        