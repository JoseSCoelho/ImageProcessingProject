%clear all;
%close all;
tic
global rotations
global translations
global camera_params

camera_params = load('calib.mat');

w_frame = 1;
max_n_points = 1000000;

% imgseq = load('midair.mat');
% imgseq = imgseq.ans;

% imgseq = load('newpiv2.mat');
% imgseq = imgseq.ans;

% imgseq = load('newpiv2.mat');
% imgseq = imgseq.ans;

% imgseq = load('short.mat');
% imgseq = imgseq.ans;
% 
imgseq = [
    struct('rgb','labpiv_unziped/rgb_image_1.png','depth','labpiv_unziped/depth_1.png')
struct('rgb','labpiv_unziped/rgb_image_2.png','depth','labpiv_unziped/depth_2.png')
struct('rgb','labpiv_unziped/rgb_image_3.png','depth','labpiv_unziped/depth_3.png')
struct('rgb','labpiv_unziped/rgb_image_4.png','depth','labpiv_unziped/depth_4.png')
struct('rgb','labpiv_unziped/rgb_image_5.png','depth','labpiv_unziped/depth_5.png')
struct('rgb','labpiv_unziped/rgb_image_6.png','depth','labpiv_unziped/depth_6.png')
struct('rgb','labpiv_unziped/rgb_image_7.png','depth','labpiv_unziped/depth_7.png')
struct('rgb','labpiv_unziped/rgb_image_8.png','depth','labpiv_unziped/depth_8.png')
struct('rgb','labpiv_unziped/rgb_image_9.png','depth','labpiv_unziped/depth_9.png')
struct('rgb','labpiv_unziped/rgb_image_10.png','depth','labpiv_unziped/depth_10.png')
struct('rgb','labpiv_unziped/rgb_image_11.png','depth','labpiv_unziped/depth_11.png')
struct('rgb','labpiv_unziped/rgb_image_12.png','depth','labpiv_unziped/depth_12.png')
struct('rgb','labpiv_unziped/rgb_image_13.png','depth','labpiv_unziped/depth_13.png')
struct('rgb','labpiv_unziped/rgb_image_14.png','depth','labpiv_unziped/depth_14.png')
struct('rgb','labpiv_unziped/rgb_image_15.png','depth','labpiv_unziped/depth_15.png')
struct('rgb','labpiv_unziped/rgb_image_16.png','depth','labpiv_unziped/depth_16.png')
struct('rgb','labpiv_unziped/rgb_image_17.png','depth','labpiv_unziped/depth_17.png')
struct('rgb','labpiv_unziped/rgb_image_18.png','depth','labpiv_unziped/depth_18.png')
struct('rgb','labpiv_unziped/rgb_image_19.png','depth','labpiv_unziped/depth_19.png')
struct('rgb','labpiv_unziped/rgb_image_20.png','depth','labpiv_unziped/depth_20.png')
struct('rgb','labpiv_unziped/rgb_image_21.png','depth','labpiv_unziped/depth_21.png')
struct('rgb','labpiv_unziped/rgb_image_22.png','depth','labpiv_unziped/depth_22.png')
          ];


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
    for j = i+1:numImgs
        if(sum(sum(abs(reshape(rotations(i, j, :, :), [3,3])))) == 0)
            Ri1 = reshape(rotations(i, 1, :, :), [3, 3]);
            Rj1 = reshape(rotations(j, 1, :, :), [3, 3]);
            R1j = reshape(rotations(1, j, :, :), [3, 3]);
            
            T1i = reshape(translations(1, i, :, :), [3, 1]);
            Ti1 = reshape(translations(i, 1, :, :), [3, 1]);
            T1j = reshape(translations(1, j, :, :), [3, 1]);
            
            % se rotação i, j estiver vazia (cala valor da matriz a 0)
            rotations(i, j, :, :) = Ri1 * R1j;
            translations(i, j, :, :) =  Ri1 * T1j + Ti1;
            %Guarda na posição transposta da matriz (transformação de 1 para j)
            rotations(j, i, :, :) = reshape(rotations(i, j, :, :), [3, 3])';
            translations(j, i, :, :) = -reshape(rotations(j, i, :, :), [3, 3]) * reshape(translations(i, j, :, :), [3, 1]);
         end
    end
end

%plot(G, 'EdgeLabel', G.Edges.Weight)
G = geraGrafo(imgseq);

%% 
pc = geraPointCloud(imgseq, G, w_frame, max_n_points);
figure();
pcshow(pc);
campos([0 0 0]);

toc
