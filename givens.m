function Q = givens(theta)

    n = length(theta);
    m = (1 + sqrt(1+8*n))/2;

    I = eye(m,class(theta));
    Q = I;
    k = 0;
    for j = 1:m-1
        for i = j+1:m
            k = k + 1;

            G = I;

            G(i,i) =  cos(theta(k));
            G(j,j) =  cos(theta(k));
            G(j,i) =  sin(theta(k));
            G(i,j) = -sin(theta(k));

            Q = Q*G;
        end
    end

end