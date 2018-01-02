%% control parameters
talkative_initialization = true;

%% tuning parameters
% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 1000;
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 10;

% patch matching with RANSAC
n_iterations_matching_RANSAC = 1000;
pixel_tolerance_RANSAC = 3; 

%% dataset specific tuning parameters
ds = 2;         % dataset - 0: KITTI, 1: Malaga, 2: parking
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

%% continuous operation
plot_tracking = true;

% Triangulate new points
alpha_deg = 5; % angle between camera views to allow triangulation
n_iterations = 1000;
pixel_tolerance =3;
min_points = 25; % minimum numbers of points required for triangulation

% parameters 
harris_patch_size_cont = 9;
harris_kappa_cont = 0.08;
num_keypoints_cont = 150;
nonmaximum_supression_radius_cont = 8;
descriptor_radius_cont = 9;
match_lambda_cont = 5;

        