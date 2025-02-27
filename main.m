clc; clear;
rng('default')
%% Parameters
L_tot = 1000;       % Number of snapshots
c0 = physconst('LightSpeed');
f0 = 2.4e9; % Operating frequency
lambda = c0/f0;      % Wavelength
d_inter = lambda/2;    % Antenna spacing (half-wavelength)
theta_min = -80;
theta_max = -theta_min;
scale_dist = 10; 
min_dist = 5; % Minimum distance of the users
ratio_vec = [0.05:0.1:0.95]; 
N_r_vec = [24];
N_t_vec = [8];
K_vec = [1];
Pt_vec = [10];
realiz_num = 100;
noise_power_dBm = -70;
K_max = 8;
dvec_main = min_dist + rand(realiz_num, K_max).*ones(realiz_num, K_max)*scale_dist;
theta_main = theta_min + rand(realiz_num, K_max).*(theta_max - theta_min);
P_res = zeros(length(K_vec), length(Pt_vec), length(N_r_vec), length(N_t_vec), length(ratio_vec), realiz_num);
RMSE_res = zeros(length(K_vec), length(Pt_vec), length(N_r_vec), length(N_t_vec), length(ratio_vec), realiz_num);
P_res_RAB = zeros(length(K_vec), length(Pt_vec), length(N_r_vec), length(N_t_vec), length(ratio_vec), realiz_num);
P_res_AA_IS = zeros(length(K_vec), length(Pt_vec), length(N_r_vec), length(N_t_vec), length(ratio_vec), realiz_num);
P_res_CSI = zeros(length(K_vec), length(Pt_vec), length(N_r_vec), length(N_t_vec), length(ratio_vec), realiz_num);
ric_fact = 100;

for nss = 1:length(K_vec)
    K = K_vec(nss);
    step = (theta_max - theta_min)/K;
    for sn = 1:length(Pt_vec)
        Pt_dB = Pt_vec(sn);     % Signal-to-Noise Ratio in dB
        P_t = 10^(Pt_dB/10);
        for nr = 1:length(N_r_vec)
            N_r = N_r_vec(nr);      % Number of receiving antennas (Ensure N_r > K for MUSIC)
            for nt = 1:length(N_t_vec)
                N_t = N_t_vec(nt);         % Number of transmit antennas 
                N_tot = N_t + N_r;  % Total number of antennas
                Kg = N_t;     % number of signals
                rng('default')
                for realiz = 1:realiz_num
                    theta = theta_main(realiz, 1:K);
                    dvec = dvec_main(realiz, 1:K);
                    refs = (randn(N_tot, K) + 1j * randn(N_tot, K)) / sqrt(2);
                    LOS_fact = sqrt(ric_fact/ (ric_fact + 1));
                    NLOS_fact = sqrt(1/ (ric_fact + 1));
                    DL_losses = (lambda./(4*pi.*dvec)).^2;
                    CRS_val = (randn + 1i * randn) / sqrt(2);
                    H_true = zeros(N_tot, K);
                    H_true_ul = zeros(N_r, K);
                    H_true_dl = zeros(N_t, K);
                    H_PK = zeros(N_tot, K);
                    theta_rad = deg2rad(theta);  % Convert DOAs to radians
                    for ii = 1:K
                        at = exp(-1j * 2 * pi * d_inter * (0:N_tot-1).' * sin(theta_rad(ii)) / lambda);
                        H_true(:, ii) = sqrt(DL_losses(ii)).*(NLOS_fact.*refs(:, ii) + LOS_fact.*at);
                    end
                    for ii = 1:K
                        at = exp(-1j * 2 * pi * d_inter * (0:N_tot-1).' * sin(theta_rad(ii)) / lambda);
                        H_PK(:, ii) = sqrt(DL_losses(ii)).*at;
                    end
                    for ii = 1:K
                        at = exp(-1j * 2 * pi * d_inter * (0:N_r-1).' * sin(theta_rad(ii)) / lambda);
                        H_true_ul(:, ii) = sqrt(DL_losses(ii)).*(NLOS_fact.*refs(N_t + 1:N_tot, ii) + LOS_fact.*at);
                    end
                    for ii = 1:K
                        at = exp(-1j * 2 * pi * d_inter * (0:N_t-1).' * sin(theta_rad(ii)) / lambda);
                        H_true_dl(:, ii) = sqrt(DL_losses(ii)).*(NLOS_fact.*refs(1:N_t, ii) + LOS_fact.*at);
                    end
                    %% Steering Matrix at Receiver (Models How Sources Arrive at Receiver)
                    A = zeros(N_r, N_t);
                    for ii = 1:K
                        A = A + CRS_val.*(H_true_ul(:, ii)*H_true_dl(:, ii).');
                    end
                    [~, P_vec_CSI] = RF_Beamforming(H_PK, P_t, H_true, L_tot);
                    [P_vec_AA_IS, P_vec_AA_RAB] = CSI_free(P_t, H_true, L_tot, theta);
                    P_res_CSI(nss, sn, nr, nt, :, realiz) = min(real(P_vec_CSI))/L_tot;
                    P_res_RAB(nss, sn, nr, nt, :, realiz) = min(P_vec_AA_RAB)/L_tot;
                    P_res_AA_IS(nss, sn, nr, nt, :, realiz) = min(P_vec_AA_IS)/L_tot;
                    for ll = 1:length(ratio_vec)
                        sense_ratio = ratio_vec(ll);
                        L_s = floor(sense_ratio*L_tot);
                        L_e = floor((1 - sense_ratio)*L_tot);
                        
                        %% Generate Independent Source Signals
                        S = (randn(Kg, L_s) + 1j * randn(Kg, L_s)) / sqrt(2);  % (K × L_s)
                        
                        %% Transmit Precoding 
                        Rx = ((P_t)/N_t).*eye(N_t); % isotropic transmission for maximum CRB
                        [Q, Lambdaa] = eig(Rx);
                        sqrt_Lambda = sqrt(Lambdaa);
                        Topt = Q * sqrt_Lambda;
                        X_t = Topt * S;  % Transmitted signals at antennas (N_t × L_s)
                        %% Compute the received powers during sensing phase
                        P_sense = zeros(K, 1);
                        for k = 1:K
                            Hk = H_true_dl(:, k)*H_true_dl(:, k)';
                            P_sense(k) = (L_s)*real(trace(Hk*Rx));
                        end
                        
                        %% Received Signal (Including Channel and Noise)
                        X = A * X_t;  % Ideal received signal (size: N_r × L_s)
                        noise_power = db2pow(noise_power_dBm - 30);
                        AWGN = sqrt(noise_power/(2)) * (randn(N_r, L_s) + 1j * randn(N_r, L_s)); % Complex AWGN
                        X = X + AWGN;  % Add noise to the received signal
                        
                        %% Run MUSIC algorithm
                        [P_MUSIC, theta_est] = MUSIC_AoA(d_inter, lambda, X, K);

                        %% Coefficient Estimation
                        alpha_hat = LS_estimate(d_inter, X_t, X, K, lambda, theta_est);
                        
                        %% Sensing Metrics and estimated channels
                        ref_dl_est = sqrt(alpha_hat);
                        H_est = zeros(N_tot, K);
                        theta_est_rad = deg2rad(theta_est);  % Convert DOAs to radians
                        for ii = 1:K
                            at = exp(-1j * 2 * pi * d_inter * (0:N_tot-1).' * sin(theta_est_rad(ii)) / lambda);
                            H_est(:, ii) = ref_dl_est(ii).*at;
                        end
                        H_est_dl = zeros(N_t, K);
                        H_est_ul = zeros(N_r, K);
                        A_est = zeros(N_r, N_t);
                        for ii = 1:K
                            at = exp(-1j * 2 * pi * d_inter * (0:N_t-1).' * sin(theta_est_rad(ii)) / lambda);
                            H_est_dl(:, ii) = ref_dl_est(ii).*at;
                        end
                        for ii = 1:K
                            at = exp(-1j * 2 * pi * d_inter * (0:N_r-1).' * sin(theta_est_rad(ii)) / lambda);
                            H_est_ul(:, ii) = ref_dl_est(ii).*at;
                        end
                        for ii = 1:K
                            A_est = A_est + (H_est_ul(:, ii)*H_est_dl(:, ii).');
                        end
                        RMSE_G = norm(A_est - A, 'fro')/sqrt(N_r*N_t);                        
                        %% RF beamforming
                        [~, P_vec] = RF_Beamforming(H_est, P_t, H_true, L_e);

                        %% Storing the results
                        final_P = real(P_vec + P_sense);
                        P_res(nss, sn, nr, nt, ll, realiz) = min(final_P)/L_tot;
                        RMSE_res(nss, sn, nr, nt, ll, realiz) = RMSE_G;
                        file_name = strcat('Targets_', string(K), '_Pt_', string(Pt_dB), '_Nr_', ...
                            string(N_r), '_Nt_', string(N_t), '_Ratio_', string(sense_ratio), '__Realiz__', string(realiz));
                        disp(file_name)
                    end
                end
            end
        end
    end
end



figure
set(gcf, 'Units', 'centimeters'); 
set(gcf, 'Position', [3 3 12 8],'PaperSize', [12 19],'PaperPositionMode','auto');
plot(ratio_vec, pow2db(reshape(mean(P_res(1, 1, 1, 1, :, :), 6), [length(ratio_vec) 1])), ...
    'LineStyle', '-', 'color',  'k','LineWidth', 2.5,'Marker', 's', 'MarkerSize',8)
hold on 
plot(ratio_vec, pow2db(reshape(mean(P_res_CSI(1, 1, 1, 1, :, :), 6), [length(ratio_vec) 1])), ...
    'LineStyle', '-', 'color',  '#0072BD','LineWidth', 2.5,'Marker', 'x', 'MarkerSize',8)
plot(ratio_vec, pow2db(reshape(mean(P_res_AA_IS(1, 1, 1, 1, :, :), 6), [length(ratio_vec) 1])), ...
    'LineStyle', '-', 'color',  '#A2142F','LineWidth', 2.5,'Marker', 'o', 'MarkerSize',8)
plot(ratio_vec, pow2db(reshape(mean(P_res_RAB(1, 1, 1, 1, :, :), 6), [length(ratio_vec) 1])), ...
    'LineStyle', '-', 'color',  '#77AC30','LineWidth', 2.5,'Marker', 'd', 'MarkerSize',8)
set(gca,'FontSize',12)
fontsize(gcf, 12,"points")
fontname(gcf, 'Times New Roman')
hold off
hl2 = legend('STC-WPT', 'PK-WPT' , 'AA-IS', 'RAB', 'Location', 'northwest');
set(hl2,'interpreter','latex','FontSize',12);
set(hl2,'color','none');
set(hl2, 'Box', 'off' );
box on
grid on
ylabel('$\frac{1}{L}min_k P_{k}$ (dB)','Interpreter', 'latex','fontsize',12)
xlabel('$\gamma$','Interpreter', 'latex','fontsize',12)

figure
set(gcf, 'Units', 'centimeters'); 
set(gcf, 'Position', [3 3 12 8],'PaperSize', [12 19],'PaperPositionMode','auto');
plot(ratio_vec, pow2db(reshape(mean(RMSE_res(1, 1, 1, 1, :, :), 6), [length(ratio_vec) 1])), ...
     'LineStyle', '-', 'color',  'k','LineWidth', 2.5,'Marker', 's', 'MarkerSize',8)
ylabel('$\mathrm{RMSE}_{\mathbf{G}_s}$','Interpreter', 'latex','fontsize',12)
xlabel('$\gamma$','Interpreter', 'latex','fontsize',12)
