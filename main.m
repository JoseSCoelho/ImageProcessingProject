clear all;
close all;

depth_array = load('short/depth_12.mat').depth_array;
camera_params = load('calib.mat');
im=imread('short/rgb_image_12.png');

PC = generate_PC(depth_array, im, camera_params);