function [diffTransl, diffRot]  = calcula_erro(j, i)
    global rotations
    global translations
    
    Rji = reshape(rotations(j, i, :, :), [3, 3]);
    Tji = reshape(translations(j, i, :, :), [3, 1]);
        
    % distancia entre as translações
    diffTransl = norm(Tji);
        
    %compares how close the matrix is to Identity
    diffRot = sum(sum(abs(Rji - eye(3))));
end