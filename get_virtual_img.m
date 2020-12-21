function [virtual_depth] = get_virtual_img(depth_array, camera_params)
    % Todos os pixeis
    [rows,cols]=size(depth_array);
    [vect_coluna,vect_linha]= meshgrid(1:cols,1:rows);

    % [u, v, 1] para cada pixel
    depth_pixel_frame = [vect_coluna(:),vect_linha(:), ones(cols*rows,1)]'; 
                            %Linhas 1 e 2 -> Pixeis; Linha 3 -> Para coordenadas homogeneas

    % Z em metros
    Z = double(depth_array(:)) / 1000; 

    % [Z*u, Z*v, Z]
    depth_pixel_frame = transpose(Z.*transpose(depth_pixel_frame));

    % [x, y, z] (refericial do depth)
    depth_frame = camera_params.Kdepth\depth_pixel_frame; %Trás as coordenadas homogeneas para 
                                            %o referencial da camera

    % [x, y, z, 1]
    depth_world = [depth_frame; ones([length(depth_frame), 1])']; %

    %%%
    % As cameras não estão alinhadas, por isso é necessário fazer a
    % correspondencia entre cada ponto de XYZ e a cor desse ponto, guardada em
    % 'im'. 'im' foi criado com o ponto de vista da camera RGB. Por isso os
    % pontos XYZ não tem correspondencia direta aos pontos da camera RGB
    % (existem pontos de XYZ sem correspondencia!)

    % RT homogenia
    rot_trans_matrix = [camera_params.R camera_params.T ; zeros(3,1)' 1];

    % [x', y', z', 1] na perspetiva da camera RGB
    rgb_world = rot_trans_matrix * depth_world; %Trás o referencial de depth para 
                                            %o referencial de RGB
    % [x', y', z'] na perspetiva da camera RGB (retira os 1s)
    rgb_frame = rgb_world(1:3, :);

    % [lambda*u', lambda*v', lambda]
    rgb_image = camera_params.Krgb*rgb_frame;

    % [u', v', lambda]
    rgb_image(1:2,:)=rgb_image(1:2,:)./rgb_image(3,:); %Divide as coordenadas x e y por lamda

    rgb_image(1:2,:)=round(rgb_image(1:2,:)); %Arredonda px e py, para serem pixeis inteiros

    % os que foram arrendondadados para 0(ou negativo) passam a 1
    %rgb_image(find(rgb_image(1:2, :)<1))=1;
    
    find_=find(rgb_image(1,:) < 1);
    rgb_image(1, find_) = 1;
    find_= find(rgb_image(2,:) < 1);
    rgb_image(2, find_) = 1;
    
    %Remove os pontos que foram captados pela camera depth, mas estao fora do
    %alcance da RGB, e arredonda para o valor valido mais proximo
    find_=find(rgb_image(1,:)>cols);
    rgb_image(1, find_) = cols;
    find_= find(rgb_image(2,:)>rows);
    rgb_image(2, find_) = rows;
    
    
%     %Cria e preenche uma imagem depth do ponto de vista da camera RGB
%     virtual_depth = zeros(rows, cols);
%     for i=1:length(rgb_image)
%         virtual_depth(rgb_image(2,i),rgb_image(1,i)) = rgb_image(3,i);
%     end

    virtual_depth = zeros(rows * cols, 1);
    virtual_depth(sub2ind([rows, cols], rgb_image(2,:),rgb_image(1,:))) = rgb_image(3,:);
    virtual_depth = reshape(virtual_depth, [rows, cols]);

end