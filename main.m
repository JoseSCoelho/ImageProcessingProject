clear all;
close all;

camera_params = load('calib.mat');
depth_array_A = load('short/depth_12.mat');
depth_array_A = depth_array_A.depth_array;
depth_array_B = load('short/depth_13.mat');
depth_array_B = depth_array_B.depth_array;
im_A=imread('short/rgb_image_12.png');
im_B=imread('short/rgb_image_13.png');

[virtual_rgb_A, virtual_depth_A] = get_virtual_img(depth_array_A, im_A, camera_params);
[virtual_rgb_B, virtual_depth_B] = get_virtual_img(depth_array_B, im_B, camera_params);

%Obtem as features e descritores de cada imagem
[frame_A, descriptor_A] = vl_sift(single(rgb2gray(im_A)));
[frame_B, descriptor_B] = vl_sift(single(rgb2gray(im_B)));
frame_A(1:2, :) = round(frame_A(1:2, :)); %Arredonda os pontos obtidos para serem
                                        %usados como pixeis
frame_B(1:2, :) = round(frame_B(1:2, :)); %Arredonda os pontos obtidos para serem
                                        %usados como pixeis                                     

    %Queremos trocar a ordem dos descriptors? Como está: os pontos da
    %imagem 12 arranjam correspondencia na imagem 13

%Obtem os pontos correspondentes entre as duas imagens
    %matches(1) --> indices dos pontos guardados no frame_12
    %matches(2) --> indices dos pontos guardados no frame_12
[matches, scores] = vl_ubcmatch(descriptor_A, descriptor_B) ;
% 
% imshow(uint8(im_A));
% hold on;
% 
% h1 = vl_plotframe(frame_A(:,matches(1, :))) ;
% h2 = vl_plotframe(frame_A(:,matches(1, :))) ;
% set(h1,'color','k','linewidth',3) ;
% set(h2,'color','y','linewidth',2) ;
% 
% figure();
% imshow(uint8(im_B));
% hold on;
% 
% h1 = vl_plotframe(frame_B(:,matches(2, :))) ;
% h2 = vl_plotframe(frame_B(:,matches(2, :))) ;
% set(h1,'color','k','linewidth',3) ;
% set(h2,'color','y','linewidth',2) ;
% 
% figure();
% imagesc(cat(2, uint8(im_A),uint8(im_B)));
% colormap(gray);
% hold on ;
% xa = frame_A(1,matches(1,:)); 
% xb = frame_B(1,matches(2,:));
% ya = frame_A(2,matches(1,:));
% yb = frame_B(2,matches(2,:));
% h = line([xa ; xb+size(im_A,2)], [ya ; yb]);
% set(h,'linewidth', 1, 'color', 'b');

%Cria 2 vetores para cada imagem com as coordenadas dos pontos obtidos pelo
%ubcmatch. Um vetor com os pontos de depth e outro de RGB
match_coord_A = frame_A(1:2, matches(1, :));
match_coord_B = frame_B(1:2, matches(2, :));

% [rows,cols]=size(virtual_depth_A);
% [vect_coluna,vect_linha]= meshgrid(1:cols,1:rows);
% pixeis_all = [vect_coluna(:), vect_linha(:)];
% figure();
% generate_PC(virtual_depth_A, pixeis_all', virtual_rgb_A, camera_params, 1);
%%
PC_A = generate_PC(virtual_depth_A, match_coord_A, virtual_rgb_A, camera_params, 0);
PC_B = generate_PC(virtual_depth_B, match_coord_B, virtual_rgb_B, camera_params, 0);

%Elimina os pontos sem correspondencia na imagem depth
% (Uni�o entre os indices dos pontos com Z==0 na PC_A e na PC_B)
remove_idx = union(find(PC_A.Location(:, 3) == 0), find(PC_B.Location(:, 3) == 0));

world = PC_A.Location;
color = PC_A.Location;
world(remove_idx, :) = [];
color(remove_idx, :) = [];
PC_A = pointCloud(world,'color', color);

world = PC_B.Location;
color = PC_B.Location;
world(remove_idx, :) = [];
color(remove_idx, :) = [];
PC_B = pointCloud(world,'color', color);



%%
%Corre o ransac, de modo a eliminar os outliers obtido pelo ubcmatch
inliers_idx = myRansac(PC_A.Location', PC_B.Location', 5000, 200);

PC_A_Inliers = generate_PC(virtual_depth_A, match_coord_A(:, inliers_idx), virtual_rgb_A, camera_params, 1);
PC_B_Inliers = generate_PC(virtual_depth_B, match_coord_B(:, inliers_idx), virtual_rgb_B, camera_params, 1);

figure();
pcshow(PC_A_Inliers);
figure();
pcshow(PC_B_Inliers);

%%
% p3 = [PC_B.Location(inliers_idx, :)'; ones(1, size(PC_B.Location(inliers_idx, :), 1))];
% rt = PC_12.Location(inliers_idx, :)' / p3;
% PC_A_estimado = (rt(:, 1:3)*PC_B.Location(inliers_idx, :)' + rt(:, 4))';    
% norms = sqrt(sum((PC_A_estimado-PC_A.Location(inliers_idx, :)).^2, 2));

%Aplica o método de procrustes de modo a obter a Rotação e Translação
[d,Z,tr] = procrustes(PC_A_Inliers.Location, ...
        PC_B_Inliers.Location, 'scaling',false, 'reflection', false);
    
Rot_Trans = [tr.T', mean(tr.c)'];

PC_A_estimado = tr.b*PC_B_Inliers.Location*tr.T(:, 1:3) +  mean(tr.c);    
 
norms = sqrt(sum((PC_A_estimado - PC_A_Inliers.Location).^2, 2));
   