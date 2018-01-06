function [hidden_state, observations, poses_boundary, all_landmarks, state_landmark_ids, num_boundary_observations] = getBAFormat(states, states_boundary, T_W_Cs, T_W_Cs_boundary)
% param states: array of structs S
% T_C_Ws 12xn transform columns
n_window = size(states,2);
% collect landmarks and transforms
states = [states states_boundary]
n = size(states,2);
twists = zeros(6*n_window,1);
poses_boundary = zeros(6*size(T_W_Cs_boundary,2),1);
X_states = [];
X_boundary = [];

for i = 1:n
    S = states(i);
    
   if i <= size(T_W_Cs,2)
       X_states = [X_states S.X];
       twist = HomogMatrix2twist([reshape(T_W_Cs(:,i),3,4); 0 0 0 1]);
       twists((i-1)*6+1:i*6) = twist;
   else
       X_boundary = [X_boundary S.X];
       ii = i-size(T_W_Cs,2);
       twist = HomogMatrix2twist([reshape(T_W_Cs_boundary(:,ii),3,4); 0 0 0 1]);
       poses_boundary((ii-1)*6+1:ii*6) = twist;
   end
end
X_all = [X_states X_boundary];
landmarks = unique(X_all', 'rows')';

X_states_unique = unique(X_states', 'rows')';

[mask, ids_states] = ismember(X_states_unique',landmarks','rows');

m = size(landmarks,2);

observations = [n;m];
num_boundary_observations = 0;
for i = 1:n
    S = states(i);
    
    if i > n_window
        num_boundary_observations = num_boundary_observations + size(S.P,2);
    end
    
    pis = S.P(:);
    ki = size(S.X,2);
    
    [mask, landmark_index] = ismember(S.X',landmarks','rows');
    lis = landmark_index(landmark_index > 0);
    assert(ki == size(lis,1));
    
    O_i = [ki; pis,; lis];
    
    observations = [observations; O_i];
end

% separatae hidden state and boundary
landmarks_state = landmarks(:,ids_states);
state_landmark_ids = ids_states;
all_landmarks = landmarks;
hidden_state = [twists; landmarks_state(:)];


