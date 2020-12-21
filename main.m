clear all;
%close all;

global rotations
global translations
global camera_params

camera_params = load('calib.mat');

w_frame = 1;
max_n_points = 1;

imgseq = load('hobbesquiet.mat');
imgseq = imgseq.ans;

% imgseq = [struct('rgb','hobbesquiet/rgb_0008.jpg','depth','hobbesquiet/depth_0008.png')
%           struct('rgb','hobbesquiet/rgb_0009.jpg','depth','hobbesquiet/depth_0009.png') ];

% imgseq = [struct('rgb','short/rgb_image_12.png','depth','short/depth_12.mat')
%           struct('rgb','short/rgb_image_13.png','depth','short/depth_13.mat')
%           struct('rgb','short/rgb_image_14.png','depth','short/depth_14.mat')
%           struct('rgb','short/rgb_image_15.png','depth','short/depth_15.mat')];

% imgseq = [struct('rgb','newpiv2/rgb_image_1.png','depth','newpiv2/depth_1.mat')
%           struct('rgb','newpiv2/rgb_image_2.png','depth','newpiv2/depth_2.mat')
%           struct('rgb','newpiv2/rgb_image_3.png','depth','newpiv2/depth_3.mat')
%           struct('rgb','newpiv2/rgb_image_4.png','depth','newpiv2/depth_4.mat')
%           struct('rgb','newpiv2/rgb_image_5.png','depth','newpiv2/depth_5.mat')
%           struct('rgb','newpiv2/rgb_image_6.png','depth','newpiv2/depth_6.mat')
%           struct('rgb','newpiv2/rgb_image_7.png','depth','newpiv2/depth_7.mat')
%           struct('rgb','newpiv2/rgb_image_8.png','depth','newpiv2/depth_8.mat')
%           struct('rgb','newpiv2/rgb_image_9.png','depth','newpiv2/depth_9.mat')
%           ];

numImgs = length(imgseq);

rotations = zeros(numImgs, numImgs, 3, 3);
translations = zeros(numImgs, numImgs, 3);

% Fills diagonal with identity
for i = 1:numImgs
    rotations(i, i, :, :) = eye(3);
end

%% Calcula as trasnformações entre imagens consecutivas (contidas em imgseq)
calcTransformacao(imgseq, [])

% Neste ponto já temos os Ri,1 e os Ri,(i+1) para as consecutivas

%% calcula cada transformação de uma imagem para a outra a partir das
% transformações de cada uma para a imagem 1
for i = 1:numImgs
    for j = 1:numImgs
        if(sum(sum(abs(reshape(rotations(i, j, :, :), [3,3])))) == 0)
            % se rotação i, j estiver vazia (cala valor da matriz a 0)
            rotations(i, j, :, :) = reshape(rotations(1, i, :, :), [3, 3])' * reshape(rotations(1, j, :, :), [3, 3]);
            translations(i, j, :, :) = reshape(translations(1, j, :, :), [3, 1]) - reshape(translations(1, i, :, :), [3, 1]);
        end
    end
end

%plot(G, 'EdgeLabel', G.Edges.Weight)
G = geraGrafo(imgseq);

%% 
pc = geraPointCloud(imgseq);
figure();
pcshow(pc);