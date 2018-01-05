function plotOverview(query_image, query_keypoints, ...
    matched_query_keypoints, matched_database_keypoints, best_inlier_mask, ...
    corresponding_landmarks, M_W_C2, n_tracked_landmarks, ground_guess)
% plots current image, trajectory of last 20 frames and landmarks, #
% tracked landmarks over last 20 frames and full trajectory


%% calculations 
run('param.m');

fig1 = figure(1); clf;
    fig1.Position = full_screen; 
    subplot(2,4,[1,2]); 
        imshow(query_image); hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
        plotMatchedKeypoints(matched_query_keypoints(:,best_inlier_mask), ...
            matched_database_keypoints(:,best_inlier_mask), 2, 'g-')
        plotMatchedKeypoints(matched_query_keypoints(:,not(best_inlier_mask)), ...
            matched_database_keypoints(:,not(best_inlier_mask)), 1, 'c-')
        title('Current Image'); 
        
    subplot(2,4,[3,4,7,8]);
        only_plot_landmarks_near_centre = false;
        frame_size = 2;
        if only_plot_landmarks_near_centre
            max_visual_distance = 25;
            landmarks_mask = (sqrt(sum(corresponding_landmarks- ...
                mean(corresponding_landmarks,2)).^2) < max_visual_distance);
        else
            landmarks_mask = logical(ones(1,size(corresponding_landmarks,2)));
        end
        query_cam_W = M_W_C2(1:3,4);
        plot3(corresponding_landmarks(1,landmarks_mask), corresponding_landmarks(2,landmarks_mask), ...
             corresponding_landmarks(3,landmarks_mask),'kx','LineWidth',2);
        hold on; 
        plot3([0,query_cam_W(1)],[0,query_cam_W(2)],[0,query_cam_W(3)],'bx-')
        plotCoordinateFrame(eye(3),zeros(3,1), frame_size);
        plotCoordinateFrame(M_W_C2(1:3,1:3),M_W_C2(1:3,4), frame_size);
        xlabel('x'); ylabel('y'), zlabel('z');
        axis equal
        rotate3d on;
        grid on
        view([0,0])
        title('Trajectory of last 20 frames and landmarks')

    subplot(2,4,5)
        plot(n_tracked_landmarks,'.-'); grid on;
        title('# tracked landmarks over last 20 frames')
        
    subplot(2,4,6)
        plot(ground_guess(1,:),ground_guess(2,:),'.-')
        title('full trajectory'); axis equal; grid on;
        xlabel('x'); ylabel('z');

end