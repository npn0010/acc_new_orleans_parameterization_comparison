function [obj,K] = pso_func(k,your_func,param_flag,l,n,m)

% create eigenvalue matrices (L)
L = cell(l,1);
c1 = 1;
c2 = n(1);
for i = 1:l
    L{i} = diag(k(c1:c2));
    if i == l
        break
    else
        c1 = c2 + 1;
        c2 = c2 + n(i+1);
    end
end
    
if length(k) > sum(n) % if matrices are full

    phi = cell(l,1);
    c1 = sum(n) + 1;
    c2 = sum(n) + m(1);
    for i = 1:l
        phi{i} = k(c1:c2);
        if i == l
            break
        else
            c1 = c2 + 1;
            c2 = c2 + m(i+1);
        end
    end

    Q = cell(l,1);
    if param_flag == 1 % Euler based approach
        for i = 1:l
            Q{i} = get_Q(n(i),phi{i});
        end
    elseif param_flag == 2 % Cayley transform
        for i = 1:l
            % x = phi{i};
            % ub = 1e3;
            % lb = -1e3;
            % y = 1/2*(lb + ub + (ub - lb)*tanh(x));
            % C = skew(y);

            C = skew(phi{i});
            I = eye(n(i));
            Q{i} = (I - C)/(I + C);
        end
    end

else % diagonal control matrices

    Q = cell(l,1);
    for i = 1:l
        Q{i} = eye(n(i));
    end

end

% perform eigen recompisition (Q*L*Q')
K = cell(1,l);
for i = 1:l
    K{i} = Q{i}*L{i}*Q{i}.';
end

obj = your_func(K{:});

end