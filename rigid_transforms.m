%clear all;
%close all;

% imgseq = load('newpiv2.mat'); imgseq = imgseq.ans;
% [f, xyz, rg] = rigid_transforms(imgseq, 2, load('calib.mat'), 500000)

function [FromCam2W, XYZ, RGB] = rigid_transforms(imgseq, w_frame, cam_params, max_n_points)
    tic
    global rotations
    global translations
    global camera_params
    
    camera_params = cam_params;

%     camera_params = load('calib.mat');
%     w_frame = 1;
%     max_n_points = 500000;
    
%     imgseq = load('midair.mat');
%     imgseq = imgseq.ans;
%     imgseq = load('newpiv2.mat');
%     imgseq = imgseq.ans;
%     imgseq = load('newpiv2.mat');
%     imgseq = imgseq.ans;
%     imgseq = load('short.mat');
%     imgseq = imgseq.ans;
    
    numImgs = length(imgseq);
    
    rotations = zeros(numImgs, numImgs, 3, 3);
    translations = zeros(numImgs, numImgs, 3);

    % Fills diagonal with identity
    for i = 1:numImgs
        rotations(i, i, :, :) = eye(3);
    end

    %% Calcula as trasnformações entre imagens consecutivas (contidas em imgseq)
    calcTransformacao(imgseq, []);

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
    [pc, FromCam2W] = geraPointCloud(imgseq, G, w_frame, max_n_points);
    figure();
    pcshow(pc);
    campos([0 0 0]);
    
    XYZ = pc.Location;
    RGB = pc.Color;
    
    toc
end