function plotOverview_cont(S, prev_S, R_C_W, t_C_W, poses, i)

run('param.m');
fig1 = figure(1); 
fig1.Position = full_screen; 

% 3d landmarks and coordinate frame
subplot(2,4,[3,4,7,8]); hold on; grid on; 
    if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
        plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
        view(0,0);
    end

    new_X = setdiff(S.X', prev_S.X', 'rows')';
    if ~isempty(new_X)
        scatter3(new_X(1, :), new_X(2, :), new_X(3, :), 5, 'r');
    end

    old_X = intersect(prev_S.X', S.X', 'rows')';
    scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'b');
    set(gcf, 'GraphicsSmoothing', 'on');
    view(0,0);
    axis equal;
    axis vis3d;
    axis(axis_array);
    xlabel('x'); ylabel('y'); zlabel('z');
    hold off

% full trajectory
figure(1); subplot(2,4,6); hold on; grid on; axis equal;      
    if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
        plot([poses(end-1,10),poses(end,10)],[poses(end-1,12),poses(end,12)],'b.-')
        title('full trajectory'); axis equal; grid on;
        xlabel('x'); ylabel('z');
    end

% keypoints statistics
subplot(2,4,5); hold on; grid on; 
    plot([i-frame_step_size,i],[prev_S.num_tracked_keypoints,S.num_tracked_keypoints],'b.-'); 
    plot([i-frame_step_size,i],[prev_S.num_added_keypoints,S.num_added_keypoints],'k.-'); 
    legend('# tracked KPs', '# added KPs','location','NE');
    xlabel('iteration');
    title('keypoints statistics')

end