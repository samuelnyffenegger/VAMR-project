%% plot ground truth
plot_animated = false; 

idx = 1:3; %size(ground_truth,1); 
if plot_animated
    figure(2); clf; hold on; 
        axis([-300,300,-100,500]); grid on;
        xlabel('x'); ylabel('y'); title('ground truth')
        for i = idx(1:end-1)
            plot(ground_truth(i:i+1,1),ground_truth(i:i+1,2),'k-');
            pause(0.001);
        end
else
    figure(1); clf; hold on; grid on;
        plot(ground_truth(idx,1),ground_truth(idx,2),'k-');    
        xlabel('x'); ylabel('y'); title('ground truth');
        axis equal
end