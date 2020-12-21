function pc = geraPointCloud(imgseq, G, w_frame, max_n_points)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global rotations
global translations
global camera_params

numImgs = length(imgseq);
grid_Avg = 0.001;
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
    virtual_depth_A = get_virtual_img(depth_array, camera_params);
    
    %Point cloud na perspetiva da camara i
    PC_orig = generate_PC(virtual_depth_A, pixeis_all', im, camera_params);
    PC_orig = pcdownsample(PC_orig, 'gridAverage', grid_Avg);
%    	PC_atual = reshape(rotations(1, i, :, :), [3, 3]) * PC_orig.Location' + reshape(translations(1, i, :, :), [3, 1]);
    % Transforma a point cloud para a perspetiva da camara world frame
    PC_atual = PC_orig.Location';
    if i ~=  w_frame
        path = shortestpath(G,i,w_frame);
        for j = 2: length(path) 
            PC_atual =  reshape(rotations(path(j), path(j-1), :, :), [3, 3]) * PC_atual + reshape(translations(path(j), path(j-1), :, :), [3, 1]);
        end 
    end
        
    % Point cloud ja na trasnformacao da camara
    pc2 = pointCloud(PC_atual', 'color', uint8([PC_orig.Color]));
    
    pc = pcmerge(pc, pc2, grid_Avg);
 
    if length(pc.Location) > max_n_points
        grid_Avg = (length(pc.Location)/max_n_points)*grid_Avg
        pc = pcdownsample(pc, 'gridAverage', grid_Avg);
    end
end
end

