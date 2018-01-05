function [T_W_Cs, states_refined] = getStandardFormat(hidden_state_optimized, observations_original, states_original, num_frames)

twists = reshape(hidden_state_optimized(1:num_frames*6), 6, []);
landmarks = reshape(hidden_state_optimized(num_frames*6+1:end), 3, []);

ptr_ki = 3; % points to first element of landmarks
n = observations_original(1);
m = observations_original(2);

T_W_Cs = zeros(12,num_frames);
states_refined = [];
for i = 1:num_frames
    T_W_C = twist2HomogMatrix(twists(:, i));
    T_W_C = T_W_C(1:3,:);
    T_W_Cs(:,i) = T_W_C(:);
    
    % get landmark ids of state
    ki = observations_original(ptr_ki);
    ptr_landmark_ids = ptr_ki + 2*ki + 1;
    landmark_ids = observations_original(ptr_landmark_ids:ptr_landmark_ids+ki-1);
    
    % get refined landmarks
    S = states_original(i);
    S.X = landmarks(:,landmark_ids);
    assert(size(S.X,2) == size(S.P,2))
    
    states_refined = [states_refined S];
    
    % point to next ki
    ptr_ki = ptr_ki + 3*ki+1;
end
end