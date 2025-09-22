function Q = cayley(phi)

    n = length(phi);
    m = (1 + sqrt(1+8*n))/2;
    
    C = zeros(m,m,class(phi));
    
    k = 0;
    for j = 1:m-1
        for i = j+1:m
            k = k + 1;
            C(j,i) =  phi(k);
            C(i,j) = -phi(k);
        end
    end

    I = eye(m,class(phi));
    Q = (I - C)/(I + C);

end