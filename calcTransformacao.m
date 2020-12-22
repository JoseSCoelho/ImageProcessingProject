function calcTransformacao(imgseq, indexes)
    %CALCTRANSFOMACAO Summary of this function goes here
    %   Detailed explanation goes here
    
    global rotations
    global translations
    global camera_params
    
    numImgs = length(imgseq);
    
    if(~isempty(indexes))
       numImgs = 2; 
    end
    
    for i = 1:(numImgs - 1)
        j = i+1;
        
        % se tivermos especificado para correr para certos indices,
        % substitui a variavel i e j
        if(~isempty(indexes))
           i = indexes(1);
           j = indexes(2);
        end
        
        disp(i)
        %analisa para a imagem i e j
        if(i == 1 || ~isempty(indexes))
            % primeira iteração
            im_A = imread(imgseq(i).rgb);
            depth_array = imread(imgseq(i).depth);


            virtual_depth_A = get_virtual_img(depth_array, camera_params);
            %Obtem as features e descritores de cada imagem
            [frame_A, descriptor_A] = vl_sift(single(rgb2gray(im_A)));
            frame_A(1:2, :) = round(frame_A(1:2, :)); %Arredonda os pontos obtidos para serem
                                                %usados como pixeis
        else
            im_A = im_B;
            virtual_depth_A = virtual_depth_B;
            frame_A = frame_B;
            descriptor_A = descriptor_B;
        end

        im_B = imread(imgseq(j).rgb);
        depth_array = imread(imgseq(j).depth);

        virtual_depth_B = get_virtual_img(depth_array, camera_params);
        [frame_B, descriptor_B] = vl_sift(single(rgb2gray(im_B)));
        frame_B(1:2, :) = round(frame_B(1:2, :));

        %Obtem os pontos correspondentes entre as duas imagens
            %matches(1) --> indices dos pontos guardados no frame_12
            %matches(2) --> indices dos pontos guardados no frame_12
        [matches, ~] = vl_ubcmatch(descriptor_A, descriptor_B) ;

        %Cria 2 vetores para cada imagem com as coordenadas dos pontos obtidos pelo
        %ubcmatch. Um vetor com os pontos de depth e outro de RGB
        match_coord_A = frame_A(1:2, matches(1, :));
        match_coord_B = frame_B(1:2, matches(2, :));

        %% Gera as pointclouds com os pontos que deram match
        PC_A = generate_PC(virtual_depth_A, match_coord_A, im_A, camera_params);
        PC_B = generate_PC(virtual_depth_B, match_coord_B, im_B, camera_params);

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
        
        %% ICP 
        procrustesRigid = affine3d([[tr.T zeros(3, 1)]; [tr.c(1, :), 1]]);
        
        icpTransf = pcregistericp(PC_B_Inliers, PC_A_Inliers, ...
            'InitialTransform', procrustesRigid, ...  %, 'Tolerance', [0.0001, 0.01]
            'Verbose', false, 'InlierRatio', 0.9);
        icpTransf = icpTransf.T';
        
        rot = icpTransf(1:3, 1:3);
        trans = icpTransf(1:3, 4);

        %Guarda na posição (i, j) a transformação de j para i 
        rotations(i, j, :, :) = rot;
        translations(i, j, :, :) = trans;
        
        %Guarda na posição (j, i) a transformação de i para j
        rotations(j, i, :, :) = rot';
        translations(j, i, :, :) = -rot'*trans;

        if(i > 1)
            %% guarda em (1, j)
            rotations(1, j, :, :) = reshape(rotations(1, i, :, :), [3, 3]) * rot;
            translations(1, j, :, :) = reshape(rotations(1, i, :, :), [3, 3]) * trans + reshape(translations(1, i, :, :), [3, 1]);
            %Guarda na posição transposta da matriz (transformação de 1 para j)
            rotations(j, 1, :, :) = reshape(rotations(1, j, :, :), [3, 3])';
            translations(j, 1, :, :) = -reshape(rotations(1, j, :, :), [3, 3])'*reshape(translations(1, j, :, :), [3, 1]);

        end
    end
end

