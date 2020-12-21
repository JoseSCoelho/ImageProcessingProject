function pc = geraPointCloud(imgseq)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global rotations
global translations
global camera_params

numImgs = length(imgseq);

% gera todas as coordenadas possíveis
[rows, cols, ~] = size(imread(imgseq(1).rgb));
[vect_coluna, vect_linha]= meshgrid(1:cols,1:rows);
pixeis_all = [vect_coluna(:), vect_linha(:)];

pc = pointCloud(double.empty(0, 3));

for i = 1:numImgs
    % Gera as point cloud completas das imagens A e B
    disp(i)
    
    % Lê a imagem i
    im = imread(imgseq(i).rgb);
    depth_array = imread(imgseq(i).depth);
    [virtual_rgb_A, virtual_depth_A] = get_virtual_img(depth_array, im, camera_params);
    PC_orig = generate_PC(virtual_depth_A, pixeis_all', virtual_rgb_A, camera_params);

    %  conjunto de pontos
    PC_estimado = reshape(rotations(1, i, :, :), [3, 3]) * PC_orig.Location' + reshape(translations(1, i, :, :), [3, 1]);
    
    % Point cloud ja na trasnformacao da camara
    pc2 = pointCloud(PC_estimado', 'color', uint8([PC_orig.Color]));
    
    pc = pcmerge(pc, pc2, 0.005);
end
end

