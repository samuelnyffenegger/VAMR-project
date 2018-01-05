function [hidden_state, observations] = getBAFormat(states, T_W_Cs)
% param states: array of structs S
% T_C_Ws 12xn transform columns
n = size(states,2);
twists = zeros(6*n,1);
% collect landmarks and transforms

X_all = [];
for i = 1:n
    S = states(i);
   X_all = [X_all S.X];
   
   twist = HomogMatrix2twist([reshape(T_W_Cs(:,i),3,4); 0 0 0 1]);
   twists((i-1)*6+1:i*6) = twist;
end
landmarks = unique(X_all', 'rows')';
m = size(landmarks,2);

hidden_state = [twists; landmarks(:)];

observations = [n;m];
for i = 1:n
    S = states(i);
    pis = S.P(:);
    ki = size(S.X,2);
    
    [mask, landmark_index] = ismember(S.X',landmarks','rows');
    lis = landmark_index(landmark_index > 0);
    assert(ki == size(lis,1));
    
    O_i = [ki; pis,; lis];
    
    observations = [observations; O_i];
end