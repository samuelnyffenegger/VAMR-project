function [S, T] = processFrame(I,I_R,S_R,K)
% Localization
KLT = vision.PointTracker % TODO: check out possible options
initialize(KLT, S_R.P', I_R);
[P_new,point_validity] = step(KLT,I);
S.X=S_R.X(:, point_validity); % only keep the points that were tracked
S.P=P_new(point_validity,:)';
[R_C_W, t_C_W, max_num_inliers_history] = ransacP3P(S.X,S.P,K);
R_C_W
t_C_W

% Triangulate new points

end
