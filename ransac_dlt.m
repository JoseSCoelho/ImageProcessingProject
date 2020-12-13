%generate points p1, and p2 from a random transformation
p1 = rand(3, 10);

T = rand(3, 1)
A = rand(3);
[u, s, v] = svd(A);
R = u*v';
if det(R) == -1
    R = -R;
end

p2 = R*p1 + T;


%%add outliers
p1 = [p1, rand(3,1), rand(3,1), rand(3,1)];
p2 = [p2, rand(3,1), rand(3,1), rand(3,1)];
p2 = [p2; ones(1, size(p2, 2))];

%tests DLT (working fine)
%rt = dlt(p1(:, 1:4), p2(:, 1:4))

% ransac to find the inliers
inliers = myRansac(p1, p2, 1000, 0.2)

[d,Z,tr] = procrustes(p1(inliers), p2(inliers));

function rt = dlt(p1, p2)
    % Returns the matrix RT that contains the transformation from p2 to p1
    rt = p1 / p2;
end

function bestInliers = myRansac(p1, p2, maxIterations, threshold)
    nPts = size(p1, 2);

    bestInliers = [];
    bestScore = 0;
    
    for i = 1:maxIterations
        rnd = randperm(nPts);         % Cria um vetor de numeros aleatórios nao repetidos
        inliersHypothesis = rnd(1:4); % escolhe 4 pontos (guarda os indices) (escolhe os primeiros 4 numeros do vetor rnd)
        ptsToTest = rnd(5:nPts);      % o resto dos pontos sao para serem testados
     
        model = dlt(p1(:, inliersHypothesis), p2(:, inliersHypothesis)); % Homography matrix
        if size(model, 1) < 3
            continue;
        end
        
        %Testa cada ponto que não os inliersHypothesis para ver se são inliers deste
        %modelo
        p2Transformed = model*p2(:, ptsToTest);
        norms = sqrt(sum((p1(:, ptsToTest)-p2Transformed).^2));
        isInlier = norms < threshold;
        
        %number of nodes with norm smalled than the treshold
        score = sum(isInlier);
        
        if score > bestScore
            bestInliers = [inliersHypothesis, ptsToTest(isInlier)]
            bestScore = score;
        end
    end
end

