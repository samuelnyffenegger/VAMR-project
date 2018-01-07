%% control parameters
talkative_initialization = true;
talkative_p3p = false;
talkative_optimization = false; 

do_plotting = true;
sliding_window_plots = true; % for keypoint statistics and landmarks
sliding_window_plots_number = 20;

% early_stopping = 0; %100; % stop after 30 frames

save_in_bags = not(do_plotting);

%% tuning parameters (tmp, should be overwritten)
% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 500; 

nonmaximum_supression_radius = 10;
descriptor_radius = 9;
match_lambda = 10;

% 8 point with RANSAC 
pixel_tolerance_8point_RANSAC = 3; 
n_iterations_8point_RANSAC = 50; 

% KLT with RANSAC
max_bidirectional_error = 1;
KLT_max_iterations = 60;
% patch size for KLT is same as harris patch size

% continuous operation
frame_step_size = 1; 

% Triangulate new points (continous operation)
min_angle_deg = 3; % angle between camera views to allow triangulation
max_angle_deg = 2.5*min_angle_deg; 
max_num_tracked_frames = 5; % only this many last frames are tracked. (old ones are not very interesting and slow down computation)

% P3P with RANSAC
discard_p3p_outliers = true; % why better performance if false?
Ransac_p3p_pixel_tolerance = 2;
estimate_DLT = false;
tweaked_for_more = false; % TODO: change this to true

%triangulation
min_points = 5; % minimum numbers of points required for triangulation

% parameters harris
harris_patch_size_cont = 9;
harris_kappa_cont = 0.08;
num_keypoints_cont = 600;
nonmaximum_supression_radius_cont = 8;
descriptor_radius_cont = 9;

% tracking
KLT_max_bidirectional_error_cont = 1.5; 
KLT_patch_size_cont = 31; %2*harris_patch_size+1; % default: 31
KLT_max_iterations_cont = 30; % default: 30
        
% tmp (should be overwritten)
axis_array = [-40 200 -10 5 -100 100];
       
% bundle adjustment
do_bundle_adjustment = true;
window_size = 20;
max_iters = 50;
boundary_window_size = 3;% must be smaller than window_size. provide "matching condition"

%% dataset specific tuning parameters
ds = 0;         % dataset - 0: KITTI, 1: Malaga, 2: parking, 

switch ds
    case 0 % Kitti parameters
        %% tuning parameters KITTI
        bootstrap_frames = [0,2]; axis_array = [-40 200 -10 5 -25 175];
        bootstrap_frames = [80,82]; axis_array = [-40 200 -10 5 -50 150];
        % bootstrap_frames = [180,182]; axis_array = [-200 40 -10 5 -75 125];

        % keypoint selection and description
        harris_patch_size = 9;
        harris_kappa = 0.08;
        num_keypoints = 300; %500; 
        nonmaximum_supression_radius = 10;
        descriptor_radius = 9;
        match_lambda = 10;

        % 8 point with RANSAC 
        pixel_tolerance_8point_RANSAC = 3; 
        n_iterations_8point_RANSAC = 50; 

        % KLT with RANSAC
        max_bidirectional_error = 1;
        KLT_max_iterations = 60;
        % patch size for KLT is same as harris patch size

        % continuous operation
        frame_step_size = 1; 

        % Triangulate new points (continous operation)
        min_angle_deg = 0.5; % angle between camera views to allow triangulation
        max_angle_deg = 3; 
        max_num_tracked_frames = 9; % only this many last frames are tracked. (old ones are not very interesting and slow down computation)

        % P3P with RANSAC
        discard_p3p_outliers = true; 
        Ransac_p3p_pixel_tolerance = 20; %5;
        estimate_DLT = false;
        tweaked_for_more = false; % TODO: change this to true
        min_points = 2; % minimum numbers of points required for triangulation

        % parameters harris
        harris_patch_size_cont = 9;
        harris_kappa_cont = 0.08;
        num_keypoints_cont = 700; 
        nonmaximum_supression_radius_cont = 8;
        descriptor_radius_cont = 9;

        % tracking
        KLT_max_bidirectional_error_cont = 6; 
        KLT_patch_size_cont = 31; %2*harris_patch_size+1; % default: 31
        KLT_max_iterations_cont = 30; % default: 30
        
                
    case 1 % Malaga parameters
%% tuning parameters malaga
bootstrap_frames = [2,4]; axis_array = [-100 20 -10 10 -30 50]; 

% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 300; %500; 
nonmaximum_supression_radius = 10;
descriptor_radius = 9;
match_lambda = 10;

% 8 point with RANSAC 
pixel_tolerance_8point_RANSAC = 3; 
n_iterations_8point_RANSAC = 50; 

% KLT with RANSAC
max_bidirectional_error = 1;
KLT_max_iterations = 60;
% patch size for KLT is same as harris patch size

% continuous operation
frame_step_size = 2; 

% Triangulate new points (continous operation)
min_angle_deg = 0.5; % angle between camera views to allow triangulation
max_angle_deg = 2.5*min_angle_deg; 
max_num_tracked_frames = 5; % only this many last frames are tracked. (old ones are not very interesting and slow down computation)

% P3P with RANSAC
discard_p3p_outliers = true; 
Ransac_p3p_pixel_tolerance = 20; %5;
estimate_DLT = false;
tweaked_for_more = false; % TODO: change this to true
min_points = 5; % minimum numbers of points required for triangulation

% parameters harris
harris_patch_size_cont = 9;
harris_kappa_cont = 0.08;
num_keypoints_cont = 300; 
nonmaximum_supression_radius_cont = 8;
descriptor_radius_cont = 9;

% tracking
KLT_max_bidirectional_error_cont = 10; 
KLT_patch_size_cont = 31; %2*harris_patch_size+1; % default: 31
KLT_max_iterations_cont = 30; % default: 30

if bootstrap_frames(1) < 2;  warning('bootstrap_frames index for malaga starting with 2'); end

    case 2 % parking parameters
        %% tuning parameters parking
        bootstrap_frames = [1,4]; axis_array = [-40 200 -10 5 -100 100];

        % keypoint selection and description
        harris_patch_size = 9;
        harris_kappa = 0.08;
        num_keypoints = 500; 
        nonmaximum_supression_radius = 10;
        descriptor_radius = 9;
        match_lambda = 10;

        % 8 point with RANSAC 
        pixel_tolerance_8point_RANSAC = 3; 
        n_iterations_8point_RANSAC = 50; 

        % KLT with RANSAC
        max_bidirectional_error = 1;
        KLT_max_iterations = 60;
        % patch size for KLT is same as harris patch size

        % continuous operation
        frame_step_size = 1; 

        % Triangulate new points (continous operation)
        min_angle_deg = 3; % angle between camera views to allow triangulation
        max_angle_deg = 2.5*min_angle_deg; 
        max_num_tracked_frames = 5; % only this many last frames are tracked. (old ones are not very interesting and slow down computation)

        % P3P with RANSAC
        discard_p3p_outliers = true; % why better performance if false?
        Ransac_p3p_pixel_tolerance = 5;
        estimate_DLT = false;
        tweaked_for_more = false; % TODO: change this to true
        min_points = 5; % minimum numbers of points required for triangulation

        % parameters harris
        harris_patch_size_cont = 9;
        harris_kappa_cont = 0.08;
        num_keypoints_cont = 300;
        nonmaximum_supression_radius_cont = 8;
        descriptor_radius_cont = 9;

        % tracking
        KLT_max_bidirectional_error_cont = 1; 
        KLT_patch_size_cont = 31; %2*harris_patch_size+1; % default: 31
        KLT_max_iterations_cont = 30; % default: 30
        
    case 3 % alpha & omega
        bootstrap_frames = [5,6];
        
    case 4 % Dübendorf, calib 1, set 1
        bootstrap_frames = [43,44];
        
    case 5 % duckietown, set 1
        
        %% tuning parameters duckietown
        % bootstrap_frames = [266,286]; axis_array =  [-40 40 -10 10 -25 125];
        % bootstrap_frames = [354,355]; % motion blurr
        % bootstrap_frames = [587,590]; axis_array = [-40 40 -10 10 -25 125];
        bootstrap_frames = [1111,1113]; axis_array = [-40 40 -10 10 -25 125];

        % keypoint selection and description
        harris_patch_size = 9;
        harris_kappa = 0.08;
        num_keypoints = 300; %500; 
        nonmaximum_supression_radius = 10;
        descriptor_radius = 9;
        match_lambda = 10;

        % 8 point with RANSAC 
        pixel_tolerance_8point_RANSAC = 3; 
        n_iterations_8point_RANSAC = 50; 

        % KLT with RANSAC
        max_bidirectional_error = 1;
        KLT_max_iterations = 60;
        % patch size for KLT is same as harris patch size

        % continuous operation
        frame_step_size = 5; 

        % Triangulate new points (continous operation)
        min_angle_deg = 3; % angle between camera views to allow triangulation
        max_angle_deg = 2.5*min_angle_deg; 
        max_num_tracked_frames = 5; % only this many last frames are tracked. (old ones are not very interesting and slow down computation)

        % P3P with RANSAC
        discard_p3p_outliers = true; 
        Ransac_p3p_pixel_tolerance = 10; %5;
        estimate_DLT = false;
        tweaked_for_more = false; % TODO: change this to true
        min_points = 5; % minimum numbers of points required for triangulation

        % parameters harris
        harris_patch_size_cont = 9;
        harris_kappa_cont = 0.08;
        num_keypoints_cont = 300; 
        nonmaximum_supression_radius_cont = 8;
        descriptor_radius_cont = 9;

        % tracking
        KLT_max_bidirectional_error_cont = 10; 
        KLT_patch_size_cont = 31; %2*harris_patch_size+1; % default: 31
        KLT_max_iterations_cont = 30; % default: 30
        
    case 6 % duckietown, set 2
        bootstrap_frames = [404,408];
                
    otherwise
        warning('choose dataset!')
end

%% screen options
set(0,'units','pixels');
full_screen = get(0,'screensize');
