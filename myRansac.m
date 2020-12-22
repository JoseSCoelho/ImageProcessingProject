function bestInliers = myRansac(p1, p2, maxIterations, threshold)
    nPts = size(p1, 2);

    bestInliers = [];
    bestScore = 0;
    
    for i = 1:maxIterations
        rnd = randperm(nPts);         % Cria um vetor de numeros aleatórios nao repetidos
        inliersHypothesis = rnd(1:4); % escolhe 4 pontos (guarda os indices) (escolhe os primeiros 4 numeros do vetor rnd)
        ptsToTest = rnd(5:nPts);      % o resto dos pontos sao para serem testados
     
        model = dlt(p1(:, inliersHypothesis), p2(:, inliersHypothesis)); 
        if(length(model) < 3)
            continue;
        end

        %Testa cada ponto que não os inliersHypothesis para ver se são inliers deste
        %modelo
        p2Transformed = model*[p2(:, ptsToTest); ones(1, length(ptsToTest))];
        norms = vecnorm(p1(:, ptsToTest)-p2Transformed); %sqrt(sum((p1(:, ptsToTest)-p2Transformed).^2));
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
    if rank(p1) < 3 || rank(p2) < 3
        rt = 0;
        return;
    end

    A = zeros(3*length(p1), 12);
    b = zeros(3*length(p1), 1);
    
    for i = 1:length(p1)
        A((i*3 - 2):(i*3), :) = [p2(1, i) p2(2, i) p2(3, i) 1 0 0 0 0 0 0 0 0;
                     0 0 0 0 p2(1, i) p2(2, i) p2(3, i) 1 0 0 0 0;
                     0 0 0 0 0 0 0 0 p2(1, i) p2(2, i) p2(3, i) 1;];
        b((i*3 - 2):(i*3)) = [p1(1, i); p1(2, i); p1(3, i)];
    end
    
    if(rcond(A' * A) < 1*10^-16)
        rt = 0;
        return
    end
    
    rt = (A'*A) \ (A'*b);
    rt = reshape(rt, [4, 3]);
    rt = rt';
end