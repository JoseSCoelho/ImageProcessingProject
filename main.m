clear all;
%close all;

camera_params = load('calib.mat');

w_frame = 1;
max_n_points = 1;
% imgseq = load('hobbesquiet.mat');
% imgseq = imgseq.ans;
% 
imgseq = [struct('rgb','short/rgb_image_12.png','depth','short/depth_12.mat')
          struct('rgb','short/rgb_image_13.png','depth','short/depth_13.mat')
          struct('rgb','short/rgb_image_14.png','depth','short/depth_14.mat')
          struct('rgb','short/rgb_image_15.png','depth','short/depth_15.mat')];

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

for i = 1:(numImgs - 1)
%     i = 1;
    disp(i)
    %analisa para a imagem i e i+1
    if(i == 1)
        im = imread(imgseq(i).rgb);
        depth_array = load(imgseq(i).depth);%imread(imgseq(i).depth);
        depth_array = depth_array.depth_array;
%         depth_array = imread(imgseq(i).depth);

        
        [virtual_rgb_A, virtual_depth_A] = get_virtual_img(depth_array, im, camera_params);
        %Obtem as features e descritores de cada imagem
        [frame_A, descriptor_A] = vl_sift(single(rgb2gray(im)));
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
%     depth_array = imread(imgseq(i+1).depth);
    
    [virtual_rgb_B, virtual_depth_B] = get_virtual_img(depth_array, im, camera_params);
    [frame_B, descriptor_B] = vl_sift(single(rgb2gray(im)));
    frame_B(1:2, :) = round(frame_B(1:2, :));

    %Obtem os pontos correspondentes entre as duas imagens
        %matches(1) --> indices dos pontos guardados no frame_12
        %matches(2) --> indices dos pontos guardados no frame_12
    [matches, scores] = vl_ubcmatch(descriptor_A, descriptor_B) ;
    
    %Cria 2 vetores para cada imagem com as coordenadas dos pontos obtidos pelo
    %ubcmatch. Um vetor com os pontos de depth e outro de RGB
    match_coord_A = frame_A(1:2, matches(1, :));
    match_coord_B = frame_B(1:2, matches(2, :));

    %% Gera as pointclouds com os pontos que deram match
    PC_A = generate_PC(virtual_depth_A, match_coord_A, virtual_rgb_A, camera_params);
    PC_B = generate_PC(virtual_depth_B, match_coord_B, virtual_rgb_B, camera_params);

    %Elimina os pontos da pointCloud sem correspondencia na imagem depth
    [PC_A, PC_B] = removeZeroPts(PC_A, PC_B);

    %% Corre o ransac, de modo a eliminar os outliers obtido pelo ubcmatch
    inliers_idx = myRansac(PC_A.Location', PC_B.Location', 5000, 0.02);

    PC_A_Inliers = pointCloud(PC_A.Location(inliers_idx, :), 'color', PC_A.Color(inliers_idx, :));
    PC_B_Inliers = pointCloud(PC_B.Location(inliers_idx, :), 'color', PC_B.Color(inliers_idx, :));

    %% Aplica o método de procrustes de modo a obter a Rotação e Translação que trasnforma do B para o A
    [d,Z,tr] = procrustes(PC_A_Inliers.Location, ...
            PC_B_Inliers.Location, 'scaling', false, 'reflection', false);
    rot = tr.T';
    trans = tr.c(1, :)';
    
    procrustesRigid = affine3d([[tr.T zeros(3, 1)]; [tr.c(1, :), 1]]);
    
    %% COM TUDO CRL
%     [rgb_A, rgb_B, wxyz_A, wxyz_B] = rgbd_to_pc(imgseq, i, i+1, camera_params);
%     
%     max_A = max(wxyz_A);
%     max_B = max(wxyz_B);
%     min_A = min(wxyz_A);
%     min_B = min(wxyz_B);
%     
%     remove_bigger = min(max_A, max_B)% + abs((max_A - max_A_esp)/20)
%     
%     
%     indexes_to_remove = find(wxyz_A(:, 1) > remove_bigger(1) | wxyz_A(:, 2) > remove_bigger(2) | wxyz_A(:, 3) > remove_bigger(3));
%     wxyz_A(indexes_to_remove, :) = [];
%     rgb_A(indexes_to_remove, :) = [];
%     
%     indexes_to_remove = find(wxyz_B(1, :) > remove_bigger(1) | wxyz_B(2, :) > remove_bigger(2) | wxyz_B(3, :) > remove_bigger(3));
%     wxyz_B(indexes_to_remove, :) = [];
%     rgb_B(indexes_to_remove, :) = [];
%     
%     
%     [icpTransf, pca_est_icp] = pcregistericp(pcdownsample(pointCloud(wxyz_B), 'gridAverage', 0.1), ...
%         pcdownsample(pointCloud(wxyz_A), 'gridAverage', 0.1), ...
%         'InitialTransform', procrustesRigid, ...  %, 'Tolerance', [0.0001, 0.01]
%         'Verbose', true, 'InlierRatio', 0.6);
%     
%     icpTransf = icpTransf.T';
%     rot = icpTransf(1:3, 1:3);
%     trans = icpTransf(1:3, 4);
    
    
%     procrustesRigid = affine3d([[tr.T zeros(3, 1)]; [tr.c(1, :), 1]]);
%     
%     
%     %     %So para ver se ta fixe
%     PC_A_estimado = tr.b * tr.T' * PC_B_Inliers.Location' + tr.c(1, :)';
%     norms_proc = sum(sqrt(sum((PC_A_estimado - PC_A_Inliers.Location').^2, 1)));
%     
%     [icpTransf, pca_est_icp] = pcregistericp(PC_B_Inliers, PC_A_Inliers, 'InitialTransform', procrustesRigid, 'Tolerance', [0.000001, 0.0001], 'Verbose', true, 'InlierRatio', 0.9);
%     %icpTransf = icpTransf.T';
%     
%     
%     
%     
%     aa = pctransform(PC_B_Inliers, icpTransf);
%     norms_icp = sum(sqrt(sum((aa.Location' - PC_A_Inliers.Location').^2, 1)));
%     
%     figure();
%     pcshow(PC_A_Inliers);
%     hold on;
%     pcshow(aa);
     
%      rot = icpTransf(1:3, 1:3);
%      trans = icpTransf(1:3, 4);
%     %icpTransf(1:3, 1:3)' *
    
%     rot = icpTransf(1:3, 1:3) * rot;
%     trans = icpTransf(1:3, 1:3) * trans + icpTransf(1:3, 4);
    

     %PC_A_estimado_ICP = rot * PC_B_Inliers.Location' + trans; % - icpTransf(1:3, 4);
%     
%     figure()
%     pcshow(PC_A_Inliers)
%     hold on;
%     pcshow(PC_A_estimado', ones(length(PC_A_estimado_ICP), 3).*[255, 0, 0])
    %pcshow(pca_est_icp.Location, ones(length(pca_est_icp.Location'), 3).*[255, 0, 0])
    
    
    rotations(i, i+1, :, :) = rot;
    translations(i, i+1, :, :) = trans;
    
    if(i > 1)
        %% guarda em (1, i+1)
        rotations(1, i+1, :, :) = rot * reshape(rotations(1, i, :, :), [3, 3]);
        translations(1, i+1, :, :) = trans + reshape(translations(1, i, :, :), [3, 1]);
    end
end

% Neste ponto já temos os Ri,1 e os Ri,(i+1) para as consecutivas

%% calcula cada transformação de uma imagem para a outra a partir das
% transformações de cada uma para a imagem 1
for i = 1:numImgs
    for j = 1:numImgs
        if(sum(sum(abs(reshape(rotations(i, j, :, :), [3,3])))) == 0)
            % se rotação i, j estiver vazia (cala valor da matriz a 0)
            rotations(i, j, :, :) = reshape(rotations(1, i, :, :), [3, 3]) * reshape(rotations(1, j, :, :), [3, 3])';
            translations(i, j, :, :) = reshape(translations(1, j, :, :), [3, 1]) - reshape(translations(1, i, :, :), [3, 1]);
        end
    end
end

%% Inicializa o grafo
%plot(G, 'EdgeLabel', G.Edges.Weight)
G = graph(zeros(numImgs));
for i = 1:(numImgs - 1)
    G = addedge(G, i, i+1, 1);
end

%% Procura as transformações parecidas

sumDiffTransl = 0;
sumDiffIdentity = 0;
threshDiffIdentity = 0;
treshTransl = 0;

for i = 2:(numImgs - 1)
    R1i = reshape(rotations(1, i, :, :), [3, 3]);
    Ti1 = reshape(translations(1, i, :, :), [3, 1]);
    
    for j = 1:(i-1)
        R1j = reshape(rotations(1, j, :, :), [3, 3]);
        T1j = reshape(translations(1, j, :, :), [3, 1]);
        
        % Rji  (confirmar?)
        a = R1j' * R1i;
        % distancia entre as translações
        diffTransl = norm(Ti1 - T1j);
        
        %compares how close the matrix is to Identity
        diffIdentity = sum(sum(abs(a - eye(3))));
        
        if (j == i-1)
            % ISTO SERVE PARA VER QUAL É UM BOM THRESHOLD. Faz uma média
            % das diferenças entre as consecutivas, para perceber o que é
            % uma transformação entre 2 imagens parecidas
            sumDiffTransl = sumDiffTransl + diffTransl;
            sumDiffIdentity = sumDiffIdentity + diffIdentity;
            
            threshDiffIdentity = sumDiffIdentity / j;
            treshTransl = sumDiffTransl / j;
        
            % ignora as imagens consecutivas
            continue;
        end
        
        
        
        if(diffIdentity < threshDiffIdentity && diffTransl < treshTransl)
            disp([i, j])
            G = addedge(G, i, j, 1);
        end
    end
end



%% 

for i = 1:(numImgs - 1)
    % Gera as point cloud completas das imagens A e B
    [rgb_A, rgb_B, wxyz_A, wxyz_B] = rgbd_to_pc(imgseq, i, i+1, camera_params);

    %  Compoe a pointCloud com as novas imagens
    wxyz_A_estimado = reshape(rotations(1, i+1, :, :), [3, 3]) * wxyz_B' + reshape(translations(1, i+1, :, :), [3, 1]);

    if (i==1)
        pc = pointCloud([wxyz_A; wxyz_A_estimado'], 'color', uint8([rgb_A; rgb_B]));
        pc = pcdownsample(pc, 'gridAverage', 0.001);
    else
        pc2 = pointCloud(wxyz_A_estimado', 'color', uint8(rgb_B));
        %pc = pointCloud([pc.Location; wxyz_A_estimado'], 'color', uint8([pc.Color; rgb_B]));
        pc = pcmerge(pc, pc2, 0.001);
        %pc = pcdownsample(pc, 'gridAverage', 0.01);
    end

    %
%     figure();
%     pcshow(pc);
end

%%
figure();
pcshow(pc);