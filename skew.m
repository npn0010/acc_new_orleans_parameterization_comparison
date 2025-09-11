function S = skew(v)
    % Compute skew-symmetric matrix size from number of parameters (i.e.,
    % length(v) = (n^2-n)/2 => n = f(length(v)) which results in a
    % quadratic equation with two roots where positive root is taken)
    n = (1 + sqrt(1+8*length(v)))/2; 
    
    S = zeros(n, n, class(v)); % Initialize the n x n matrix skew-symmetric matrix
    
    % Fill in the upper triangular part
    k = 1; % Index for v elements
    for i = 1:n-1
        for j = i+1:n
            S(i, j) = v(k);  % Upper triangular part
            S(j, i) = -v(k); % Lower triangular part (negative)
            k = k + 1;
        end
    end
end