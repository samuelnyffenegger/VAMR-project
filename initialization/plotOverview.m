function plotOverview(query_image, query_keypoints, ...
    matched_query_keypoints, matched_database_keypoints, best_inlier_mask)
% plots current image, trajectory of last 20 frames and landmarks, #
% tracked landmarks over last 20 frames and full trajectory


%% calculations 
clc

% dummy
t = 0:0.01:2*pi; y = sin(t);

figure(1); clf
    subplot(2,4,[1,2]); 
        imshow(query_image); hold on;
        plot(query_keypoints(2, :), query_keypoints(1, :), 'rx', 'Linewidth', 2);
%         plotMatches(all_matches, query_keypoints, database_keypoints, 1, 'k-')
        plotMatchedKeypoints(matched_query_keypoints(:,best_inlier_mask), ...
        matched_database_keypoints(:,best_inlier_mask), 2, 'g-')
        title('Current Image'); 
        
    subplot(2,4,[3,4,7,8]);
        plot(t,y)
        title('Trajectory of last 20 frames and landmarks')
        
    subplot(2,4,5)
        plot(t,y)
        title('# tracked landmarks over last 20 frames')
    subplot(2,4,6)
        plot(t,y)
        title('full trajectory')


end