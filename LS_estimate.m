%% Least Square estimation 

function refs_est = LS_estimate(d, X_t, X, K, lambda, theta)

N_t = size(X_t, 1);
N_r = size(X, 1);   
A_t = zeros(N_t, K);
A_r = zeros(N_r, K);
M = size(X, 2); 
for k = 1:K
    A_t(:, k) = exp(-1j * 2 * pi * d * (0:N_t-1)' * sin(deg2rad(theta(k)))/ lambda);
    A_r(:, k) = exp(-1j * 2 * pi * d * (0:N_r-1)' * sin(deg2rad(theta(k)))/ lambda);
end
B = A_t.' * X_t;
Bvar = kron(B.', A_r);
myalph = inv(Bvar'*Bvar)*Bvar'*reshape(X, [N_r*M 1]);
refs_est = diag(reshape(myalph, [K K])); 

end