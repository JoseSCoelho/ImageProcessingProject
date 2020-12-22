function new_pc = generate_PC (depth_array, match_coord, im, camera_params)

%     %Lista de das cores para cada ponto
%     pc_color=ones(length(match_coord),3);
%     for i=1:length(match_coord)
%         pc_color(i,:)=im(match_coord(2,i),match_coord(1,i),:);
%     end

    height = size(im, 1);
    width = size(im, 2);
    % Lista de das cores para cada ponto
    pc_color = zeros(length(match_coord), 3);
    flat_im = reshape(im, [width*height*3, 1]);

    pc_color(:, 1) = flat_im(sub2ind([height, width, 3], match_coord(2, :), match_coord(1, :), ones(1, length(match_coord))*1));
    pc_color(:, 2) = flat_im(sub2ind([height, width, 3], match_coord(2, :), match_coord(1, :), ones(1, length(match_coord))*2));
    pc_color(:, 3) = flat_im(sub2ind([height, width, 3], match_coord(2, :), match_coord(1, :), ones(1, length(match_coord))*3));

    
    %Passa a os pontos rgb_match paras as suas coordenadas espaciais
    depth_matches = zeros(length(match_coord), 1);
%     for i=1:length(match_coord)
%         depth_matches(i)=depth_array(match_coord(2,i),match_coord(1,i),:);
%     end
    
    flat_depth = reshape(depth_array, [height*width, 1]);
    depth_matches(:, 1) = flat_depth(sub2ind([height, width], match_coord(2, :), match_coord(1, :)));

    world = (camera_params.Krgb\[match_coord.*depth_matches' ; depth_matches'])';
    
    new_pc = pointCloud(world,'color', uint8(pc_color));
end 