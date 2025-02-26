# STC-WPT
This repository provides the MATLAB simulation codes for the paper: "Sense-then-charge Protocol for RF Wireless Power Transfer to Unresponsive Devices with Unknown Location".
## CSI-free.m
This file generates the results for the benchmark CSI-free approaches: RAB [ref1] and AA-IS [ref2].

[ref1]: O. L. A. López, H. Alves, S. Montejo-Sánchez, R. D. Souza and M. Latva-aho, "CSI-Free Rotary Antenna Beamforming for Massive RF Wireless Energy Transfer," in IEEE Internet of Things Journal, vol. 9, no. 10, pp. 7375-7387, 15 May 2022, doi: 10.1109/JIOT.2021.3107222.

[ref2]: O. L. A. López, S. Montejo-Sánchez, R. D. Souza, C. B. Papadias and H. Alves, "On CSI-Free Multiantenna Schemes for Massive RF Wireless Energy Transfer," in IEEE Internet of Things Journal, vol. 8, no. 1, pp. 278-296, 1 Jan.1, 2021, doi: 10.1109/JIOT.2020.3003114.

## MUSIC_AoA.m
This file includes the MUSIC algorithm [ref3] for estimating the AoAs.

[ref3]: R. Schmidt, “Multiple emitter location and signal parameter estimation,” IEEE Trans. Antennas Propag., vol. 34, no. 3, pp. 276–280, 1986.
## LS_estimate.m
This file performs the least square estimation on the received signal based on the obtained AoAs to estimate the reflection matrix coefficients.
## RF_Beamforming.m
This file solves the WPT beamforming problem using semi-definite programming and CVX [ref4].

[ref4]: M. Grant and S. Boyd, “CVX: Matlab software for disciplined convex programming, version 2.1.” http://cvxr.com/cvx, Mar. 2014.
## main.m
This file includes an example of a main script, which generates the results for a given setup and plots the figures of RMSE and minimum received power as a function of the sensing allocation coefficient.
