%%

% match figure with matlab windows
set(0,'units','pixels')

fig1 = figure(1); clf;
fig1.Position = get(0,'screensize'); 


%% 
clc
T_W_C = reshape(poses(end,:),3,4)

-R_C_W'*t_C_W
poses(end,10:12)'
