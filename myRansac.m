function bestInliers = myRansac(p1, p2, maxIterations, threshold)
    nPts = size(p1, 2);

    bestInliers = [];
    bestScore = 0;
    
    for i = 1:maxIterations
        rnd = randperm(nPts);         % Cria um vetor de numeros aleatórios nao repetidos
        inliersHypothesis = rnd(1:4); % escolhe 4 pontos (guarda os indices) (escolhe os primeiros 4 numeros do vetor rnd)
        ptsToTest = rnd(5:nPts);      % o resto dos pontos sao para serem testados
     
        model = dlt(p1(:, inliersHypothesis), p2(:, inliersHypothesis)); 
        if(size(model) < 3)
            continue;
        end

%         [d,Z,tr] = procrustes(p1(:, inliersHypothesis)', ...
%             p2(:, inliersHypothesis)', 'scaling', false, 'reflection', false);
%         rot = tr.T';
%         trans = tr.c(1, :)';
%         
%         model = [rot trans];
%         
        
        
        %Testa cada ponto que não os inliersHypothesis para ver se são inliers deste
        %modelo
        p2Transformed = model*[p2(:, ptsToTest); ones(1, length(ptsToTest))];
        norms = sqrt(sum((p1(:, ptsToTest)-p2Transformed).^2));
        isInlier = norms < threshold;
        
        %number of nodes with norm smalled than the treshold
        score = sum(isInlier);
        
        if score > bestScore
            bestInliers = [inliersHypothesis, ptsToTest(isInlier)];
            bestScore = score;
        end
    end
end

function rt = dlt(p1, p2)
    % Returns the matrix RT that contains the transformation from p2 to p1
    if rank(p1) < 3 || rank(p2) < 3 || rcond(p1'*p1) < 10^(-18) || rcond(p2'*p2) < 10^(-18)
        rt = 0;
    else
%         rt = p1 / [p2; ones(1, length(p2))];
        rt = [p2; ones(1, length(p2))]' \ p1';
        rt = rt';
    end
end