%% post processing plots 2d
% ground truth (2d plot)
fig5 = figure(5); clf;  hold on; zoom on
    fig5.Position = full_screen;
    plot(poses(:,10),poses(:,12),'b-*','linewidth',3);
    axis equal; grid on; xlabel('x'); ylabel('z')
    plot(landmarks(1,:),landmarks(3,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    xlabel('x'); ylabel('z');

    fig6 = figure(6); clf;  hold on; zoom on
    fig6.Position = full_screen;
    set(gca,'Ydir','reverse')
    plot(poses(:,10),poses(:,11),'b-*','linewidth',3);
    plot(landmarks(1,:),landmarks(2,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    axis equal; grid on; xlabel('x'); ylabel('y (watch direction)')


fig7 = figure(7); clf;  hold on; grid on; 
    fig7.Position = full_screen;
    plot([bootstrap_frames(2),range(1:size(num_keypoints_statistics,2)-1)],num_keypoints_statistics);
    xlabel('image frame number')
    legend('# tracked KPs','# added KPs','location','NE'); 

   
%% post processing plots 3d
fig8 = figure(8); clf;  hold on; 
    fig8.Position = full_screen;
    plot3(poses(:,10),poses(:,11),poses(:,12),'b-*','linewidth',3);
    axis equal; grid on; xlabel('x'); ylabel('z')
    plot(landmarks(1,:),landmarks(3,:),'k.','linewidth',0.5)
    title('estimated ground truth and keypoints')
    xlabel('x'); ylabel('z');
