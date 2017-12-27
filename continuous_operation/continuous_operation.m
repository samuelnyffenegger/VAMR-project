clc;
close all;
clear;

S = struct('P',[],'X',[],'C',[],'F',[],'T',[]);
 S.P = load('/Users/Sebastian/Documents/MATLAB/VisionAlgorithms/Project/code/../..//Ex6/data/keypoints.txt')';
 S.P =[S.P(2,:);S.P(1,:)]
 S.X = load('/Users/Sebastian/Documents/MATLAB/VisionAlgorithms/Project/code/../..//Ex6/data/p_W_landmarks.txt')';
 K = load('/Users/Sebastian/Documents/MATLAB/VisionAlgorithms/Project/code/../..//Ex6/data/K.txt');
figure(1);

scatter3(S.X(1, :), S.X(2, :), S.X(3, :), 5);
set(gcf, 'GraphicsSmoothing', 'on');
view(0,0);
axis equal;
axis vis3d;
axis([-40 10 -10 5 -1 60]);
for i=0:8
    prev_I = imread(['/Users/Sebastian/Documents/MATLAB/VisionAlgorithms/Project/code/../../Ex6/data/' sprintf('%06d.png',i)]);
    I = imread(['/Users/Sebastian/Documents/MATLAB/VisionAlgorithms/Project/code/../../Ex6/data/' sprintf('%06d.png',i+1)]);
    hold on
    [S_new, R_C_W, t_C_W] = processFrame(I,prev_I,S, K);
    plot = true;
    if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)  
        plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
        view(0,0); 
    end
    S=S_new;
    hold off

end
