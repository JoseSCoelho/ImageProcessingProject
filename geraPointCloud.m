function [pc, FromCam2W] = geraPointCloud(imgseq, G, w_frame, max_n_points)
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

FromCam2W = repmat(struct('R', 0, 'T', 0), numImgs, 1);

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
    
    
    PC_atual = PC_orig.Location';
    
    %% Transforma a point cloud para a perspetiva da camara world frame
    % Transformation to world_frame
    rot = eye(3);
    trans = zeros(3, 1);
    
    
    % Cria a transformação para a world_frame juntando as trasnformações
    % entre os saltos calculados no caminho mais curto
    if i ~=  w_frame
        path = shortestpath(G, i, w_frame);
        for j = 2: length(path) 
            rot = reshape(rotations(path(j), path(j-1), :, :), [3, 3]) * rot;
            trans = reshape(rotations(path(j), path(j-1), :, :), [3, 3]) * trans + reshape(translations(path(j), path(j-1), :, :), [3, 1]);
        end
        
        PC_atual = rot * PC_atual + trans;
    end
    
    FromCam2W(i).R = rot;
    FromCam2W(i).T = trans;
    
        
    % Point cloud ja na trasnformacao da camara
    pc2 = pointCloud(PC_atual', 'color', uint8([PC_orig.Color]));
    
    %% merge das PCs
    pc = pcmerge(pc, pc2, grid_Avg);
 
    if length(pc.Location) > max_n_points
        grid_Avg = (length(pc.Location)/max_n_points)*grid_Avg
        pc = pcdownsample(pc, 'gridAverage', grid_Avg);
    end
end

% downsample Final para garantir que n. pontos está abaixo de max_n_points
percentage_to_remove = max_n_points / pc.Count;
if(percentage_to_remove < 1)
    pc = pcdownsample(pc, 'random', percentage_to_remove);
end

end