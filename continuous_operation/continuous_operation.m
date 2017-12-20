S_i = struct('P',0,'X',0,'C',0,'F',0,'T',0)
I_R = imread('../../../Ex6/data/000000.png');
I = imread('../../../Ex6/data/000001.png');
points = detectSURFFeatures(I);
S_i.P = load('../../../Ex6/data/keypoints.txt')';
S_i.X = load('../../../Ex6/data/p_W_landmarks.txt')';
K = load('../../../Ex6/data/K.txt');
processFrame(I,I_R,S_i, K);