clear all;
%close all;

camera_params = load('calib.mat');

w_frame = 1;
max_n_points = 1;

imgseq = load('midair.mat');
imgseq = imgseq.ans;

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

for i = 1:(numImgs - 1)
%     i = 1;
    disp(i)
    %analisa para a imagem i e i+1
    if(i == 1)
        im = imread(imgseq(i).rgb);
%         depth_array = load(imgseq(i).depth);%imread(imgseq(i).depth);
%         depth_array = depth_array.depth_array;
        depth_array = imread(imgseq(i).depth);

        
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
%     depth_array = load(imgseq(i+1).depth);
%     depth_array = depth_array.depth_array;
    depth_array = imread(imgseq(i+1).depth);
    
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
    inliers_idx = myRansac(PC_A.Location', PC_B.Location', 500, 0.02);

    PC_A_Inliers = pointCloud(PC_A.Location(inliers_idx, :), 'color', PC_A.Color(inliers_idx, :));
    PC_B_Inliers = pointCloud(PC_B.Location(inliers_idx, :), 'color', PC_B.Color(inliers_idx, :));

    %% Aplica o método de procrustes de modo a obter a Rotação e Translação que trasnforma do B para o A
    [d,Z,tr] = procrustes(PC_A_Inliers.Location, ...
            PC_B_Inliers.Location, 'scaling', false, 'reflection', false);
    rot = tr.T';
    trans = tr.c(1, :)';
        
    rotations(i, i+1, :, :) = rot;
    translations(i, i+1, :, :) = trans;
    
    if(i > 1)
        %% guarda em (1, i+1)
        rotations(1, i+1, :, :) = reshape(rotations(1, i, :, :), [3, 3]) * rot;
        translations(1, i+1, :, :) = reshape(rotations(1, i, :, :), [3, 3]) * trans + reshape(translations(1, i, :, :), [3, 1]);
    end
end

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

% TODO: ler da matriz

for i = 2:(numImgs - 1)
    R1i = reshape(rotations(1, i, :, :), [3, 3]);
    T1i = reshape(translations(1, i, :, :), [3, 1]);
    
    for j = 1:(i-1)
        R1j = reshape(rotations(1, j, :, :), [3, 3]);
        T1j = reshape(translations(1, j, :, :), [3, 1]);
        
        % Rji  (confirmar?)
        a = R1j' * R1i;
        % distancia entre as translações
        diffTransl = norm(T1i - T1j);
        
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

% gera todas as coordenadas possíveis
[rows,cols]=size(virtual_depth_A);
[vect_coluna,vect_linha]= meshgrid(1:cols,1:rows);
pixeis_all = [vect_coluna(:), vect_linha(:)];

for i = 1:(numImgs - 1)
    % Gera as point cloud completas das imagens A e B
%    [rgb_A, rgb_B, wxyz_A, wxyz_B] = rgbd_to_pc(imgseq, i, i+1, camera_params);
    disp(i)
%     
    
    if(i == 1)
        % Lê a imagem i
        im = imread(imgseq(i).rgb);
%         depth_array = load(imgseq(i).depth);
%         depth_array = depth_array.depth_array;
        depth_array = imread(imgseq(i).depth);
        [virtual_rgb_A, virtual_depth_A] = get_virtual_img(depth_array, im, camera_params);
        wxyz_A_PC = generate_PC(virtual_depth_A, pixeis_all', virtual_rgb_A, camera_params);
    else
        % Passa a PC B para a A
        wxyz_A_PC = wxyz_B_PC;
    end
    % Lê a imagem i+1
    im = imread(imgseq(i+1).rgb);
%     depth_array = load(imgseq(i+1).depth);
%     depth_array = depth_array.depth_array;
    depth_array = imread(imgseq(i+1).depth);
    [virtual_rgb_B, virtual_depth_B] = get_virtual_img(depth_array, im, camera_params);
    wxyz_B_PC = generate_PC(virtual_depth_B, pixeis_all', virtual_rgb_B, camera_params);
    

    %  Compoe a pointCloud com as novas imagens
    wxyz_A_estimado = reshape(rotations(1, i+1, :, :), [3, 3]) * wxyz_B_PC.Location' + reshape(translations(1, i+1, :, :), [3, 1]);
%     wxyz_A_estimado = reshape(rotations(1, i+1, :, :), [3, 3]) * wxyz_B' + reshape(translations(1, i+1, :, :), [3, 1]);

    if (i==1)
        pc = pointCloud([wxyz_A_PC.Location; wxyz_A_estimado'], 'color', uint8([wxyz_A_PC.Color; wxyz_B_PC.Color]));
%         pc = pointCloud([wxyz_A; wxyz_A_estimado'], 'color', uint8([rgb_A; rgb_B]));
        pc = pcdownsample(pc, 'gridAverage', 0.001);
    else
%         pc2 = pointCloud(wxyz_A_estimado', 'color', uint8(rgb_B));
%         pc = pcmerge(pc, pc2, 0.001);
        pc = pointCloud([pc.Location; wxyz_A_estimado'], 'color', uint8([pc.Color; wxyz_B_PC.Color]));
        pc = pcdownsample(pc, 'gridAverage', 0.001);
    end

    %
%     figure();
%     pcshow(pc);
end

figure();
pcshow(pc);