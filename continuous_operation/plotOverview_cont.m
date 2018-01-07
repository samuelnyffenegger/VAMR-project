function plotOverview_cont(S, prev_S, R_C_W, t_C_W, poses, i,landmarks_container, refined_poses)

run('param.m');
fig1 = figure(1); 
fig1.Position = full_screen; 

tic
% full trajectory
figure(1); subplot(2,4,6); hold on; grid on; axis equal;      
    if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
        plot([poses(end-1,10),poses(end,10)],[poses(end-1,12),poses(end,12)],'b.-')
        if ~isempty(refined_poses)
            plot(refined_poses(10,:),refined_poses(12,:),'g.-')
            legend('normal','refined with BA','location','NW')
        end
        title('full trajectory'); axis equal; grid on;

        xlabel('x'); ylabel('z');
    end
toc
tic
% keypoints statistics
subplot(2,4,5); hold on; grid on; 
    plot([i-frame_step_size,i],[prev_S.num_tracked_keypoints,S.num_tracked_keypoints],'b.-'); 
    plot([i-frame_step_size,i],[prev_S.num_added_keypoints,S.num_added_keypoints],'k.-'); 
    legend('# tracked KPs', '# added KPs','location','NE');
    xlabel('iteration');
    title('keypoints statistics')
toc
% 3d landmarks and coordinate frame
tic
sp2 = subplot(2,4,[3,4,7,8]); hold on; grid on;
    if sliding_window_plots
        cla(sp2);
        sp2 = subplot(2,4,[3,4,7,8]); hold on; grid on;
        
        X = cell2mat(landmarks_container);
        % landmarks
        
        if ~isempty(X)
                scatter3(X(1,:), X(2,:), X(3,:), 5, 'k'); 
        end
         
        for i = 1:min(sliding_window_plots_number,size(poses,1))
            % camera coordinate frame
            T_W_C = reshape(poses(i,:),3,4);
            R_W_C = T_W_C(1:3,1:3); t_W_C = T_W_C(1:3,4);
            T_C_W = [R_W_C', -R_W_C'*t_W_C];
            R_C_W = T_C_W(1:3,1:3); t_C_W = T_C_W(1:3,4);
            
            plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
            
            
        end
        position_cam = -R_C_W'*t_C_W; 
            axis([position_cam(1)-20, position_cam(1)+20, ...
                  position_cam(2)-10, position_cam(2)+10,...
                  position_cam(3)-20, position_cam(3)+40])
        view([0,0]);
        %set(gcf, 'GraphicsSmoothing', 'on'); view(0,0);
        axis equal; axis vis3d;
        xlabel('x'); ylabel('y'); zlabel('z');
        title('Trajectory of last 20 frames and landmarks')
            
    else
        if all(size(R_C_W) > 0) && all(size(t_C_W) > 0)
            plotCoordinateFrame(R_C_W', -R_C_W'*t_C_W, 2);
%            view(0,0);
        end

        new_X = setdiff(S.X', prev_S.X', 'rows')';
        if ~isempty(new_X)
            scatter3(new_X(1, :), new_X(2, :), new_X(3, :), 5, 'b');
        end

        scatter3(prev_S.X(1, :), prev_S.X(2, :), prev_S.X(3, :), 5, 'k');
    end
    toc
end