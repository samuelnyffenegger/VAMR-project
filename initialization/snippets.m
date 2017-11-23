%% fundamtental matrix using normalized 8 point algorithm with RANSAC
clear all; close all; clc;
load stereoPointPairs

[fRANSAC,inliersIndex,status] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4)

% inlierPts1 = matchedPoints1(knownInliers,:);
% inlierPts2 = matchedPoints2(knownInliers,:);
% fNorm8Point = estimateFundamentalMatrix(inlierPts1,inlierPts2,'Method','Norm8Point')


