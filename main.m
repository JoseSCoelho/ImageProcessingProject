clear all;
close all;

camera_params = load('calib.mat');

w_frame = 1;
max_n_points = 1;
%imgseq = load('midair.mat');
%imgseq = imgseq.ans;

imgseq = [struct('rgb','short/rgb_image_13.png','depth','short/depth_13.mat')
          struct('rgb','short/rgb_image_14.png','depth','short/depth_14.mat')];

rotations = zeros(length(imgseq), length(imgseq), 3, 3);
translations = zeros(length(imgseq), length(imgseq), 3);

%for i = 1:(length(imgseq) - 1)
    i = 1;
    disp(i)
    %analisa para a imagem i e i+1
    if(i == 1)
        im = imread(imgseq(i).rgb);
        depth_array = load(imgseq(i).depth);%imread(imgseq(i).depth);
        depth_array = depth_array.depth_array;
        
        [virtual_rgb_A, virtual_depth_A] = get_virtual_img(depth_array, im, camera_params);
        %Obtem as features e descritores de cada imagem
        [frame_A, descriptor_A] = vl_sift(single(rgb2gray(im)), 'edgethresh', 300);
        frame_A(1:2, :) = round(frame_A(1:2, :)); %Arredonda os pontos obtidos para serem
                                            %usados como pixeis
    else
        virtual_rgb_A = virtual_rgb_B;
        virtual_depth_A = virtual_depth_B;
        frame_A = frame_B;
        descriptor_A = descriptor_B;
    end
    
    im = imread(imgseq(i+1).rgb);
    depth_array = load(imgseq(i+1).depth);
    depth_array = depth_array.depth_array;
    %depth_array = imread(imgseq(i+1).depth);
    
    [virtual_rgb_B, virtual_depth_B] = get_virtual_img(depth_array, im, camera_params);
    [frame_B, descriptor_B] = vl_sift(single(rgb2gray(im)), 'edgethresh', 300);
    frame_B(1:2, :) = round(frame_B(1:2, :));

    %Obtem os pontos correspondentes entre as duas imagens
        %matches(1) --> indices dos pontos guardados no frame_12
        %matches(2) --> indices dos pontos guardados no frame_12

    [matches, scores] = vl_ubcmatch(descriptor_A, descriptor_B) ;
    
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
    PC_A = generate_PC(virtual_depth_A, match_coord_A, virtual_rgb_A, camera_params);
    PC_B = generate_PC(virtual_depth_B, match_coord_B, virtual_rgb_B, camera_params);

    %Elimina os pontos da pointCloud sem correspondencia na imagem depth
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
    inliers_idx = myRansac(PC_A.Location', PC_B.Location', 500, 200);

    PC_A_Inliers = pointCloud(PC_A.Location(inliers_idx, :), 'color', PC_A.Color(inliers_idx, :));
    PC_B_Inliers = pointCloud(PC_B.Location(inliers_idx, :), 'color', PC_B.Color(inliers_idx, :));

%     figure();
%     pcshow(PC_A_Inliers);
%     hold on;
%     pcshow(PC_B_Inliers);

    %%
    %Aplica o método de procrustes de modo a obter a Rotação e Translação que trasnforma do B para o A
    [d,Z,tr] = procrustes(PC_A_Inliers.Location, ...
            PC_B_Inliers.Location, 'scaling', true, 'reflection', false);
    rot = tr.T';
    trans = tr.c(1, :)';
    
    %So para ver se ta fixe
    PC_A_estimado = tr.b * rot * PC_B_Inliers.Location' + trans;
    norms = sqrt(sum((PC_A_estimado - PC_A_Inliers.Location').^2, 1));
    
    % a = reshape(rotations(1, 2, :, :), [3, 3])
    
    rotations(i, i+1, :, :) = rot;
    translations(i, i+1, :, :) = trans;
    
    %%
    
    [rgb_A, rgb_B, wxyz_A, wxyz_B] = rgbd_to_pc(imgseq, 1, 2, camera_params);
    
    %%
    wxyz_A_estimado = rot * wxyz_B' + trans;
    
%     
%     centroid_A = sum(wxyz_A)/length(wxyz_A);
%     centroid_A_estimado = sum(wxyz_A_estimado')/length(wxyz_A_estimado');
%     
%     trans_centroid = (centroid_A - centroid_A_estimado)/2;
%    
%     wxyz_A_estimado = rot * wxyz_B' + trans_centroid';
    
%     icpTransf = pcregistericp(pcdownsample(pointCloud(wxyz_A),'random', 0.1), pcdownsample(pointCloud(wxyz_A_estimado'),'random', 0.1));
%     icpTransf = icpTransf.T';
%     
%     rot = icpTransf(1:3, 1:3)' * rot;
%     trans = icpTransf(1:3, 4) + trans;
%     
%     wxyz_A_estimado = rot * wxyz_B' + trans;
    
%     pcshow(wxyz_A, uint8(rgb_A));
%     hold on;
    a = [255, 0, 0] .* ones(length(wxyz_A_estimado), 3);
    
    %pcshow(wxyz_A_estimado', uint8(a));
    
    
    pc = pointCloud([wxyz_A; wxyz_A_estimado'], 'color', uint8([rgb_A; a]));
    pc = pcdownsample(pc,'random', 0.5);
    
    figure();
    pcshow(pc);
%end