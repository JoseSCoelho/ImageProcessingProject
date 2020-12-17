function [virtual_rgb, virtual_depth] = get_virtual_img(depth_array, im, camera_params)
    [rows,cols]=size(depth_array);

    [vect_coluna,vect_linha]= meshgrid(1:cols,1:rows);

    depth_pixel_frame = [vect_coluna(:),vect_linha(:), ones(cols*rows,1)]'; 
                            %Linhas 1 e 2 -> Pixeis; Linha 3 -> Para coordenadas homogeneas

    Z = double(depth_array(:)) / 1000; 

    depth_pixel_frame = transpose(Z.*transpose(depth_pixel_frame));

    depth_frame = camera_params.Kdepth\depth_pixel_frame; %Trás as coordenadas homogeneas para 
                                            %o referencial da camera

    depth_world = [depth_frame; ones([length(depth_frame), 1])']; %

    %%%
    % As cameras não estão alinhadas, por isso é necessário fazer a
    % correspondencia entre cada ponto de XYZ e a cor desse ponto, guardada em
    % 'im'. 'im' foi criado com o ponto de vista da camera RGB. Por isso os
    % pontos XYZ não tem correspondencia direta aos pontos da camera RGB
    % (existem pontos de XYZ sem correspondencia!)

    rot_trans_matrix = [camera_params.R camera_params.T ; zeros(3,1)' 1];

    rgb_world = rot_trans_matrix\depth_world; %Trás o referencial de depth para 
                                            %o referencial de RGB

    rgb_frame = rgb_world(1:3, :);

    rgb_image = camera_params.Krgb*rgb_frame;

    rgb_image(1,:)=rgb_image(1,:)./rgb_image(3,:); %Divide coordenada x por lamda
    rgb_image(2,:)=rgb_image(2,:)./rgb_image(3,:); %Divide coordenada y por lamda

    rgb_image(1:2,:)=round(rgb_image(1:2,:)); %Arredonda px e py, para serem pixeis inteiros

    %Remove os pontos que foram captados pela camera depth, mas estao fora do
    %alcance da RGB, e arredonda para o valor valido mais proximo
    rgb_image(find(rgb_image<1))=1;
    find_=find(rgb_image(1,:)>cols);
    rgb_image(1, find_)=cols;
    find_= find(rgb_image(2,:)>rows);
    rgb_image(2, find_)=rows;

    %Cria e preenche uma imagem depth do ponto de vista da camera RGB
    virtual_depth = zeros(rows, cols);
    for i=1:length(rgb_image)
        virtual_depth(rgb_image(2,i),rgb_image(1,i))=rgb_image(3,i);
    end
    
    %Cria e preenche uma imagem RGB com os pontos fornecidos pela camera depth
    virtual_rgb = zeros(rows, cols, 3);
    for i=1:length(rgb_image)
        virtual_rgb(rgb_image(2,i),rgb_image(1,i),:)=im(rgb_image(2,i), rgb_image(1,i),:);
    end
%     figure()
%     imshow(uint8(virtual_rgb));
%     figure()
%     imagesc(virtual_depth);
    %Corta os pixeis sem informação?????
    
end