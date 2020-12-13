clear all;
close all;

camera_params = load('calib.mat');
depth_array_12 = load('short/depth_12.mat').depth_array;
depth_array_13 = load('short/depth_13.mat').depth_array;
im_12=imread('short/rgb_image_12.png');
im_13=imread('short/rgb_image_13.png');

%Obtem as features e descritores de cada imagem
[frame_12, descriptor_12] = vl_sift(single(rgb2gray(im_12)));
[frame_13, descriptor_13] = vl_sift(single(rgb2gray(im_13)));
frame_12(1:2, :) = round(frame_12(1:2, :)); %Arredonda os pontos obtidos para serem
                                        %usados como pixeis
frame_13(1:2, :) = round(frame_13(1:2, :)); %Arredonda os pontos obtidos para serem
                                        %usados como pixeis                                     

    %Queremos trocar a ordem dos descriptors? Como está: os pontos da
    %imagem 12 arranjam correspondencia na imagem 13

%Obtem os pontos correspondentes entre as duas imagens
    %matches(1) --> indices dos pontos guardados no frame_12
    %matches(2) --> indices dos pontos guardados no frame_12
[matches, scores] = vl_ubcmatch(descriptor_12, descriptor_13) ;

%Cria 2 vetores para cada imagem com as coordenadas dos pontos obtidos pelo
%ubcmatch. Um vetor com os pontos de depth e outro de RGB
match_coord_12 = frame_12(1:2, matches(1, :));
match_coord_13 = frame_13(1:2, matches(2, :));

PC_12 = generate_PC(depth_array_12(match_coord_12(2, :), match_coord_12(1, :)), im_12(match_coord_12(2, :), match_coord_12(1, :), :), camera_params);
PC_13 = generate_PC(depth_array_13(match_coord_13(2, :), match_coord_13(1, :), :), im_13(match_coord_13(2, :), match_coord_13(1, :), :), camera_params);

%Corre o ransac, de modo a eliminar os outliers obtido pelo ubcmatch
inliers_idx = myRansac(PC_12.Location', PC_13.Location', 5000, 0.2);

p3 = [PC_13.Location(inliers_idx, :)'; ones(1, size(PC_13.Location(inliers_idx, :), 1))];

rt = PC_12.Location(inliers_idx, :)' / p3;

PC_12_estimado = (rt(:, 1:3)*PC_13.Location(inliers_idx, :)' + rt(:, 4))';    
 
norms = sqrt(sum((PC_12_estimado-PC_12.Location(inliers_idx, :)).^2, 2));
%%
%Aplica o método de procrustes de modo a obter a Rotação e Translação
[d,Z,tr] = procrustes(PC_12.Location(inliers_idx, :), ...
        PC_13.Location(inliers_idx, :), 'scaling',false); %, 'reflection', false
    
Rot_Trans = [tr.T', mean(tr.c)'];

PC_12_estimado = tr.b*PC_13.Location(inliers_idx, :)*tr.T(:, 1:3) +  mean(tr.c);    
 
norms = sqrt(sum((PC_12_estimado - PC_12.Location(inliers_idx, :)).^2, 2))
   
%Sera que a matriz de rotação está transpo
 
