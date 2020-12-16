function new_pc = generate_PC (depth_array, match_coord, im, camera_params, var)

    %Lista de das cores para cada ponto
    pc_color=ones(length(match_coord),3);
    for i=1:length(match_coord)
        if(var == 1)
            pc_color(i,:)=im(match_coord(2,i),match_coord(1,i),:);
        else
            pc_color(i,:)=[255, 0, 0];
        end
    end
    
    %Passa a os pontos rgb_match paras as suas coordenadas espaciais
    depth_matches = zeros(length(match_coord), 1);
    for i=1:length(match_coord)
        depth_matches(i)=depth_array(match_coord(2,i),match_coord(1,i),:);
    end
    world = (camera_params.Krgb\[match_coord.*depth_matches' ; depth_matches'])';
    
    new_pc = pointCloud(world,'color', uint8(pc_color));
end 