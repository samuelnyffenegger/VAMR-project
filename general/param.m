%% control parameters
talkative_initialization = true;

%% tuning parameters
% keypoint selection and description
harris_patch_size = 9;
harris_kappa = 0.08;
num_keypoints = 600; %better tracking implemented (KLT and non max supp)
nonmaximum_supression_radius = 8;
descriptor_radius = 9;
match_lambda = 10;

% patch matching with RANSAC
n_iterations_matching_RANSAC = 50;
pixel_tolerance_RANSAC = 3; 

% KLT with RANSAC
max_bidirectional_error = 1;
KLT_max_iterations = 30;
% patch size for KLT is same as harris patch size

%% dataset specific tuning parameters
ds = 2;         % dataset - 0: KITTI, 1: Malaga, 2: parking
switch ds
    case 0 % Kitti parameters
        bootstrap_frames = [0,2]; 
        % bootstrap_frames = [100,102];
        % bootstrap_frames = [200,202];
       
    case 1 % Malaga parameters
        bootstrap_frames = [0,2]; 

    case 2 % parking parameters
        bootstrap_frames = [40,45]; 
    
    case 3 % alpha & omega
        bootstrap_frames = [5,6];
        
    case 4 % Dübendorf, calib 1, set 1
        bootstrap_frames = [43,44];
        
    case 5 % duckietown, set 1
        bootstrap_frames = [266,267];
        bootstrap_frames = [354,355]; % motion blurr
        bootstrap_frames = [587,590]; % turn right
        bootstrap_frames = [1111,1113]; % turn right
        
    case 6 % duckietown, set 2
        bootstrap_frames = [404,408];
                
    otherwise
        warning('choose dataset!')
end

%% continuous operation
plot_tracking = true;

% Triangulate new points
alpha_deg = 5; % angle between camera views to allow triangulation
n_iterations_triangulation = 50;
pixel_tolerance =2;
min_points = 25; % minimum numbers of points required for triangulation

% parameters 
harris_patch_size_cont = 9;
harris_kappa_cont = 0.08;
num_keypoints_cont = 300;
nonmaximum_supression_radius_cont = 10;
descriptor_radius_cont = 9;

        
