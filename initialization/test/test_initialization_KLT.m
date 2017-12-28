%% test initialization_KLT

clear all; clc;

database_image = rgb2gray(imread('star1.png'));
query_image = rgb2gray(imread('star2.png'));
[H, W] = size(database_image);
database_image = database_image(70:H-70, 110:W-110);
query_image = query_image(70:H-70, 110:W-110);


figure(1); clf;
    imshow(database_image)
    pause(1)
    imshow(query_image)

