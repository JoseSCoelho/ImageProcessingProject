function G = geraGrafo(imgseq)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global rotations
global translations

numImgs = length(imgseq);
    
G = graph(zeros(numImgs));
% for i = 1:(numImgs - 1)
%     G = addedge(G, i, i+1, 1);
% end

%% Procura as transforma��es parecidas

sumDiffTransl = 0;
sumDiffRot = 0;
threshDiffIdentity = 0;
treshTransl = 0;

for i = 1:(numImgs - 1)
    [diffTransl, diffRot] = calcula_erro(i, i+1);

    % ISTO SERVE PARA VER QUAL � UM BOM THRESHOLD. Faz uma m�dia
    % das diferen�as entre as consecutivas, para perceber o que �
    % uma transforma��o entre 2 imagens parecidas
    sumDiffTransl = sumDiffTransl + diffTransl;
    sumDiffRot = diffRot + diffRot;
    
    %Acrescenta uma aresta entre 2 imagens consecutivas
    G = addedge(G, i, i+1, diffRot*diffTransl);
end

threshDiffIdentity = sumDiffRot / (numImgs - 1) * 1.2;
treshTransl = sumDiffTransl / (numImgs - 1);

for i = 2:(numImgs)
    for j = 1:(i-1)
        
        [diffTransl, diffRot] = calcula_erro(j, i);
        
        if (j == i-1)
            % ignora as imagens consecutivas
            continue;
        end
        
        if(diffRot < threshDiffIdentity && diffTransl < treshTransl)
            % Descobriu um caminho melhor (j) <- (i) (j � sempre menor)
            disp([i, j])
            
            % atualiza as rotations e tranlations do i para o j e o inverso
            calcTransformacao(imgseq, [i, j])
            
            % Recalcula o peso para a nova transforma��o obtida
            [diffTransl, diffRot] = calcula_erro(j, i);
            G = addedge(G, i, j, diffRot*diffTransl);
        end
    end
end

end