function [W, P_vec] = RF_Beamforming(H_est, P_t, H, L_e)

K = size(H, 2);
P_vec = zeros(K, 1);
N_t = size(H, 1);

%% WPT beamforming based on estimated channels

if K == 1 % for single -user, MRT is optimal
    hk = H_est(:, 1);
    wk = sqrt(P_t).*(hk/norm(hk));
    W = wk*wk';
else
    cvx_begin sdp quiet
        variable W(N_t, N_t) hermitian semidefinite
        variable t
        maximize t
        for kk = 1:K
            hk = H_est(:, kk);
            Hk = hk*hk';
            t <= trace(W*Hk);
        end
        trace(W) <= P_t
    cvx_end
end

%% Compute actual received power

for kk = 1:K
    hk = H(:, kk);
    Hk = hk*hk';
    P_vec(kk) = (L_e)*trace(W*Hk);
end

end