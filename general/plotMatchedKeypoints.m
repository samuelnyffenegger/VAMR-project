function plotMatchedKeypoints(matched_query_keypoints, matched_database_keypoints, Linewidth, Colour)
% plotMatches(matches, query_keypoints, database_keypoints);
% plots green line between matches
% query keypoint i corresponds to database keypoint i
% Input:
%   matched_query_keypoints, MxQ
%   matched_database_keypoints, MxQ

if nargin < 4; Colour = 'g-'; end
if nargin < 3; Linewidth = 2; end

x_from = matched_query_keypoints(1, :);
x_to = matched_database_keypoints(1, :);
y_from = matched_query_keypoints(2, :);
y_to = matched_database_keypoints(2, :);
plot([y_from; y_to], [x_from; x_to], Colour, 'Linewidth', Linewidth);

end

