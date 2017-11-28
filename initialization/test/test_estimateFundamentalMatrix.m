clear all % clear all variables in the workspace
close all % close all figures
clc       % clear the command window

rng(42);

N = 40;         % Number of 3-D points
X = randn(4,N);  % Homogeneous coordinates of 3-D points

% Simulated scene with error-free correspondences
X(3, :) = X(3, :) * 5 + 10;
X(4, :) = 1;

K = [500 0 320 
     0 500 240
     0 0 1]; 

R1 = eye(3); t1 = zeros(3,1); 
P1 =   K*[R1,t1]; % origing

R2 = eye(3); t2 = [-0.2;0;0]; % 
P2 =   K*[R2,t2];
				
x1 = P1 * X;     % Image (i.e., projected) points
x2 = P2 * X;
x1_ = [x1(1,:);x1(2,:)]./x1(3,:);
x2_ = [x2(1,:);x2(2,:)]./x2(3,:);

sigma = 1e-1;
noisy_x1 = x1 + sigma * randn(size(x1));
noisy_x2 = x2 + sigma * randn(size(x1));

noisy_x1_ = [noisy_x1(1,:);noisy_x1(2,:)]./noisy_x1(3,:);
noisy_x2_ = [noisy_x2(1,:);noisy_x2(2,:)]./noisy_x2(3,:);

%% Fundamental matrix estimation via the 8-point algorithm

% Estimate fundamental matrix
% Call the 8-point algorithm on inputs x1,x2
F = fundamentalEightPoint(x1,x2);
F_ = estimateFundamentalMatrix(x1_',x2_','Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);

% Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
cost_algebraic = norm( sum(x2.*(F_*x1)) ) / sqrt(N);
cost_dist_epi_line = distPoint2EpipolarLine(F_,x1,x2);

fprintf('Noise-free correspondences\n');
fprintf('Algebraic error: %f\n', cost_algebraic);
fprintf('Geometric error: %f px\n\n', cost_dist_epi_line);

%% Test with noise:

% Estimate fundamental matrix
% Call the 8-point algorithm on noisy inputs x1,x2
F = fundamentalEightPoint(noisy_x1,noisy_x2)
[F_, inliers_mask, status] = estimateFundamentalMatrix(noisy_x1_',noisy_x2_','Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4); F_

fprintf('n_inliers = %2i \n',num2str(nnz(inliers_mask)))

% Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
cost_algebraic = [norm( sum(noisy_x2.*(F*noisy_x1)) ) / sqrt(N); ...
                  norm( sum(noisy_x2.*(F_*noisy_x1)) ) / sqrt(N)];
cost_dist_epi_line = [distPoint2EpipolarLine(F,noisy_x1,noisy_x2); ...
                      distPoint2EpipolarLine(F_,noisy_x1,noisy_x2)];

fprintf('Noisy correspondences (sigma=%f), with fundamentalEightPoint\n', sigma);
printmat([cost_algebraic,cost_dist_epi_line],'cost','F F_','alg dist')


%% Normalized 8-point algorithm
% Call the normalized 8-point algorithm on inputs x1,x2
Fn = fundamentalEightPoint_normalized(noisy_x1,noisy_x2);
Fn_ = estimateFundamentalMatrix_normalized(noisy_x1,noisy_x2,2000,1e-4);

% Check the epipolar constraint x2(i).' * F * x1(i) = 0 for all points i.
cost_algebraic = [norm( sum(noisy_x2.*(Fn*noisy_x1)) ) / sqrt(N); ...
                  norm( sum(noisy_x2.*(Fn_*noisy_x1)) ) / sqrt(N);];
cost_dist_epi_line = [distPoint2EpipolarLine(Fn,noisy_x1,noisy_x2); ...
                      distPoint2EpipolarLine(Fn_,noisy_x1,noisy_x2);]

fprintf('Noisy correspondences (sigma=%f), with fundamentalEightPoint_normalized\n', sigma);
printmat([cost_algebraic,cost_dist_epi_line],'cost','F F_','alg dist')

