function [P_vec_AA_IS, P_vec_AI_RAB] = CSI_free(P_t, H, L, theta_deg)

K = size(H, 2);
N_t = size(H, 1);
theta = deg2rad(theta_deg);
P_vec_AA_IS = zeros(K, 1);
P_vec_AI_RAB = zeros(K, 1);
M = N_t;
N = K;

%% AA-IS strategy

for kk = 1:K
    hk = H(:, kk);
    P_vec_AA_IS(kk) = L*(P_t/N_t)*norm(hk)^2;
end


%% RAB strategy

        
%LOS channel generation for each device
h_rot = zeros(N,M,M);
for i=1:N
    %For the rotary mechanism, the LOS channel changes for each beam
    %direction m
    for m=1:M
        phi_rot = -(0:(M-1))*pi*sin(theta(i)+pi*m/M)+mod(0:(M-1),2)*pi;
        h_rot(i,:,m) = exp(1i*pi/4)*exp(1i*phi_rot);                 
    end
end
                
%RAB performance for each EH device
for i=1:K
    hk = H(:, i);
    E=(abs(hk(1))^2).*abs(sum(squeeze(h_rot(i,:,:))./sqrt(M))).^2;
    P_vec_AI_RAB(i) = L*P_t*mean(E);  % without power control
end


end