function Q = get_Q(n,phi)

% organize angles into matrix and assign angle range
phi_matrix = zeros(n - 1,n - 1);
starting_idx = 1;
for i = 1:n-1
    phi_matrix(1:n-i,i) = pi*phi(starting_idx:starting_idx + n - i - 1);
    temp_ind = length(phi(starting_idx:starting_idx + n - i - 1)); % last angle in the current column of matrix
    phi_matrix(temp_ind,i) = 2*phi_matrix(temp_ind,i);
    starting_idx = starting_idx + n - i;
end

% construct first unit vector
unit_vector = zeros(n,1);
temp_phi_ind = 1;
% Loop to compute the first n-1 components
for i = 1:n-1
    % Calculate the product of the sines for the current component
    prod_sin = 1;
    for j = 1:i-1
        prod_sin = prod_sin * sin(phi_matrix(temp_phi_ind,j));
    end
    % Multiply by the cosine of the current angle
    unit_vector(i) = prod_sin * cos(phi_matrix(temp_phi_ind,i));
end
% Compute the nth component
prod_sin = 1;
for j = 1:n-1
    prod_sin = prod_sin * sin(phi_matrix(temp_phi_ind,j));
end
unit_vector(n) = prod_sin;
Q = unit_vector;

for ii = n-1:-1:1
    temp_phi_ind = temp_phi_ind + 1;
    v_null = null(Q.');
    unit_vector = zeros(ii,1);
    % Loop to compute the first n-1 components
    for i = 1:ii-1
        % Calculate the product of the sines for the current component
        prod_sin = 1;
        for j = 1:i-1
            prod_sin = prod_sin * sin(phi_matrix(temp_phi_ind,j));
        end
        % Multiply by the cosine of the current angle
        unit_vector(i) = prod_sin * cos(phi_matrix(temp_phi_ind,i));
    end
    % Compute the nth component
    prod_sin = 1;
    for j = 1:ii-1
        prod_sin = prod_sin * sin(phi_matrix(temp_phi_ind,j));
    end
    unit_vector(ii) = prod_sin;

    Q = [Q v_null*unit_vector];

end

end