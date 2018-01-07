function hidden_state = runBA(hidden_state, observations, poses_boundary, all_landmarks, state_landmark_ids, num_boundary_observations, K, max_iters)
% Update the hidden state, encoded as explained in the problem statement,
% with 20 bundle adjustment iterations.
% param matching_observations: these observations enter into error function
% but cannot be refined

with_pattern = true; % setting this to false just slows everything down very much
boundary_window_size = size(poses_boundary,1)/6;

if with_pattern
    num_frames_window = observations(1)-boundary_window_size;
    num_frames_all = observations(1);
    num_observations = (numel(observations)-2-num_frames_all)/3;
    
    % Factor 2, one error for each x and y direction.
    num_error_terms_window = 2 * (num_observations-num_boundary_observations);
    num_error_terms_boundary = 2 * num_boundary_observations;
    % Each window error term will depend on one pose (6 entries) and one landmark
    % position (3 entries), so 9 nonzero entries per window error term.
    % Each boundary error term depends only on one or zero landmarks (3 entries).
    pattern = spalloc(num_error_terms_window + num_error_terms_boundary, numel(hidden_state), ...
        num_error_terms_window * 9 + num_error_terms_boundary*3);
    
    % Fill pattern for each frame individually:
    observation_i = 3;  % iterator into serialized observations
    error_i = 1;  % iterating frames, need another iterator for the error
    for frame_i = 1:num_frames_all
        num_keypoints_in_frame = observations(observation_i);
        
        
        if frame_i <= num_frames_window
            % All errors of a window frame are affected by its pose.
            pattern(error_i:error_i+2*num_keypoints_in_frame-1, ...
                (frame_i-1)*6+1:frame_i*6) = 1;
        end
        
        % Each error is then also affected by the corresponding landmark,
        % if the landmark is part of the set to be optimized.
        landmark_indices = observations(...
            observation_i+2*num_keypoints_in_frame+1:...
            observation_i+3*num_keypoints_in_frame);
        for kp_i = 1:numel(landmark_indices)
            % can only change landmarks that are visible from the window
            % states
            if ismember(kp_i, state_landmark_ids)
                %have to shift id accordingly, since landmarks only visible in
                %boundary are not changed
                num_non_optimized_before = landmark_indices(kp_i) - find(state_landmark_ids == landmark_indices(kp_i));
                pattern(error_i+(kp_i-1)*2:error_i+kp_i*2-1,...
                    1+num_frames_window*6+(landmark_indices(kp_i)-1-num_non_optimized_before)*3:...
                    num_frames_window*6+(landmark_indices(kp_i)-num_non_optimized_before)*3) = 1;
            end
        end        
        observation_i = observation_i + 1 + 3*num_keypoints_in_frame;
        error_i = error_i + 2 * num_keypoints_in_frame;
    end
end

% Use an external error function for clean code.
error_terms = @(x) baErrorWindow(x, observations, poses_boundary, all_landmarks, state_landmark_ids,K);
options = optimoptions(@lsqnonlin, 'Display', 'iter', ...
    'MaxIter', max_iters);
if with_pattern
    options.JacobPattern = pattern;
    options.UseParallel = false;
end

% run optimization
hidden_state = lsqnonlin(error_terms, hidden_state, [], [], options);

end

 