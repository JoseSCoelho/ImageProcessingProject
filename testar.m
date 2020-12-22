% clear all;
close all;

imgseq = load('newpiv2.mat');
imgseq = imgseq.ans;

max_n_points = 500000;

% corre funcao como o stor quer
[f, xyz, rgb] = rigid_transforms(imgseq, 2, load('calib.mat'), max_n_points);

pc = pointCloud(xyz, 'color', rgb);
figure();
pcshow(pc);
campos([0 0 0]);