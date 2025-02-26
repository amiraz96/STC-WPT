function [P_MUSIC, theta_est] = MUSIC_AoA(d, lambda, X, K)

N_r = size(X, 1);
M = size(X, 2);
%% Compute Covariance Matrix
Rxx = (X * X') / M;  % Sample covariance matrix (N_r × N_r)

%% Eigenvalue Decomposition
[E, D] = eig(Rxx);  % Eigen-decomposition
[~, idx] = sort(diag(D), 'descend'); % Sort eigenvalues in descending order
E = E(:, idx);  % Sort eigenvectors accordingly

%% Determine Noise and Signal Subspaces
Es = E(:, 1:K);        % Signal subspace (first K eigenvectors)
En = E(:, K+1:end);    % Noise subspace (remaining eigenvectors)

%% MUSIC Spectrum Estimation
theta_scan = -90:0.1:90;  % Scanning angles
P_MUSIC = zeros(size(theta_scan));  % Initialize spectrum

for i = 1:length(theta_scan)
    a_theta = exp(-1j * 2 * pi * d * (0:N_r-1).' * sin(deg2rad(theta_scan(i))) / lambda);  % Steering vector (N_r × 1)
    P_MUSIC(i) = 1 / (a_theta' * (En * En') * a_theta); % MUSIC formula
end

%% Normalize and Convert to dB
P_MUSIC = abs(P_MUSIC);
P_MUSIC_dB = 10 * log10(P_MUSIC / max(P_MUSIC));
% Extract the K largest values and their indices
[peaks, locations] = findpeaks(P_MUSIC_dB);
[~, sorted_indices] = sort(peaks, 'descend');
sorted_locations = locations(sorted_indices);
theta_est = [];
for ss = 1:min(K, length(sorted_locations))
    theta_est(ss) = theta_scan(sorted_locations(ss));
end
theta_est = sort(theta_est, 'ascend');

%% if there isn't sufficient peaks, just add the main direction as user AoA
while true
    if length(theta_est) < K
        theta_est(end + 1) = 0;
    else
        break
    end
end


end
