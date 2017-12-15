%% test script which changes M (3x4) into a trajectory
% M is M_1_i (not M_i-1_i)
clc


poses = load([kitti_path,'/poses/00.txt']);
% poses = load([parking_path '/poses.txt']);


n_frames = size(poses,1);
M_0_0 = reshape(poses(1,:),4,3)';

figure(1);clf; hold on;
M_1_i = reshape(poses(1,:),4,3)';
for i = 2:2%n_frames
    M_old = M_1_i; 
    M_1_i = reshape(poses(i,:),4,3)';
    fprintf('M_%i_%i = \n',1,i); disp(M_1_i)

    plot([M_old(1,4),M_1_i(1,4)], ...
        [M_old(3,4),M_1_i(3,4)],'b.-')
    axis equal; grid on; 
    axis([-272,293,-18,478]); %kitti
    
    pause(0.0001)

end
