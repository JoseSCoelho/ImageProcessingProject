function G = geraGrafo(imgseq)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global camera_params
global rotations
global translations

numImgs = length(imgseq);
    
G = graph(zeros(numImgs));
for i = 1:(numImgs - 1)
    G = addedge(G, i, i+1, 1);
end

%% Procura as transforma��es parecidas

sumDiffTransl = 0;
sumDiffIdentity = 0;
threshDiffIdentity = 0;
treshTransl = 0;

for i = 2:(numImgs - 1)
    for j = 1:(i-1)
        Rji = reshape(rotations(j, i, :, :), [3, 3]);
        Tji = reshape(translations(j, i, :, :), [3, 1]);
        
        % distancia entre as transla��es
        diffTransl = norm(Tji);
        
        %compares how close the matrix is to Identity
        diffIdentity = sum(sum(abs(Rji - eye(3))));
        
        if (j == i-1)
            % ISTO SERVE PARA VER QUAL � UM BOM THRESHOLD. Faz uma m�dia
            % das diferen�as entre as consecutivas, para perceber o que �
            % uma transforma��o entre 2 imagens parecidas
            sumDiffTransl = sumDiffTransl + diffTransl;
            sumDiffIdentity = sumDiffIdentity + diffIdentity;
            
            threshDiffIdentity = sumDiffIdentity / j;
            treshTransl = sumDiffTransl / j;
        
            % ignora as imagens consecutivas
            continue;
        end
        
        if(false && diffIdentity < threshDiffIdentity*1.1 && diffTransl < treshTransl*1.1)
            % Descobriu um caminho melhor (j) <- (i) (j � sempre menor)
            disp([i, j])
            G = addedge(G, i, j, 1);
            
            % atualiza as rotations e tranlations do i para o j e o inverso
            calcTransformacao(imgseq, [i, j])
        end
    end
end
end