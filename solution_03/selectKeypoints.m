function keypoints = selectKeypoints(scores, num, r)
% keypoints = selectKeypoints(scores, num, r); 
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.
% Input:
%   scores, size(img)
%   num, scalar number of best scores where a non-maxima supression is executed
%   r, scalar "radius" of supression square
% Output:
%   keypoints, 2xnum, coorinates of keypoint

keypoints = zeros(2, num);
temp_scores = padarray(scores, [r r]);
for i = 1:num
    [~, kp] = max(temp_scores(:));
    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [row;col];
    keypoints(:, i) = kp - r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end

end

