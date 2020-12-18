function [PC_A,PC_B] = removeZeroPts(PC_A,PC_B)
%REMOVEZEROPTS Elimina os pontos da pointCloud sem correspondencia na imagem depth
% (União entre os indices dos pontos com Z==0 na PC_A e na PC_B)
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
end

