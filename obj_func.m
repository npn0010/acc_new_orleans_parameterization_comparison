function [obj,K] = obj_func(k,your_func,K_flag,l,m,n)

% create eigenvalue matrices (L)
Lambda = cell(l,1); % preallocate eigenvalue matrices
c1 = 1; % first index counter
c2 = m(1); % second index counter
for i = 1:l % loop over number of control matrices
    Lambda{i} = diag(k(c1:c2)); % create diagonal eigenvalue matrix
    if i ~= l % update index counters (except on last iteration)
        c1 = c2 + 1; % update first index counter
        c2 = c2 + m(i+1); % update second index counter
    end
end

Q = cell(l,1); % preallocate eigenvector matrices

% create orthogonal eigenvector matrices
if K_flag == 1 % diagonal control matrices

    for i = 1:l % loop over number of control matrices
        Q{i} = eye(m(i)); % eigenvector matrices are identity for diagonal control matrices
    end

else % full matrices

    theta = cell(l,1); % preallocate orthgonal matrix parameterization variables
    c1 = sum(m) + 1; % first index counter
    c2 = sum(m) + n(1); % second index counter
    for i = 1:l % loop over number of control matrices
        theta{i} = k(c1:c2); % extract vector of variables associated with i-th control matrix
        if i ~= l % update index counters
            c1 = c2 + 1; % update first index counter
            c2 = c2 + n(i+1); % update second index counter
        end
    end

    for i = 1:l % loop over number of control matrices
        if K_flag == 2 % givens rotation parameterization
            Q{i} = givens(theta{i});
        elseif K_flag == 3 % cayley transform parameterization
            Q{i} = cayley(theta{i});
        elseif K_flag == 4 % cayley transform parameterization with mapping
            Q{i} = cayley(tan(theta{i}/2));
        end
    end
end

K = cell(1,l); % preallocate control matrices
for i = 1:l % loop over number of control matrices
    K{i} = Q{i}*Lambda{i}*Q{i}.'; % perform eigen recompisition (Q*L*Q')
end

obj = your_func(K{:}); % evaluate objective function

end