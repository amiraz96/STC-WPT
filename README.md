# Sense-Then-Charge Wireless Power Transfer Simulations

This repository contains MATLAB implementations for the simulations described in "Sense-then-charge: Wireless Power Transfer to Unresponsive Devices with Unknown Location". The code models a multi-user RF wireless power transfer (RF-WPT) system that first senses the propagation channel and then applies optimized beamforming to maximize the minimum received power across users.

## Repository contents

| File | Purpose |
| --- | --- |
| `main.m` | End-to-end experiment driver. Sweeps system parameters, runs sensing, estimation, and beamforming, and stores metrics such as minimum received power and several RMSE measures. |
| `RF_Beamforming.m` | Solves the downlink WPT beamforming problem using maximal ratio transmission for single-user cases and a semidefinite program (via CVX) for multi-user scenarios. |
| `MUSIC_AoA.m` | Implements the MUSIC algorithm to estimate user angles of arrival from received uplink pilot snapshots. |
| `LS_estimate.m` | Performs least-squares estimation of complex reflection coefficients given the estimated angles and the sensed transmit/receive signals. |
| `CSI_free.m` | Evaluates channel-state-information-free benchmark strategies: adaptive antenna with isotropic sensing (AA-IS) and rotary antenna beamforming (RAB). |

## Requirements

* MATLAB (R2021a or later recommended).
* CVX 2.1 or newer with an SDP-compatible solver (e.g., SDPT3, SeDuMi) for the semidefinite program in `RF_Beamforming.m`.
* MATLAB Phased Array System Toolbox (for `physconst`).

## Running the main experiment

1. Install CVX and run `cvx_setup` inside MATLAB.
2. Clone or download this repository and add the folder to the MATLAB path (for example, `addpath(genpath('path/to/STC-WPT'))`).
3. Open `main.m` and adjust the simulation settings as desired. Key parameters include:
   * `ratio_vec`: fraction of coherence blocks allocated to sensing versus charging.
   * `N_r_vec` / `N_t_vec`: numbers of receiver and transmitter antennas.
   * `K_vec`: number of devices.
   * `Pt_vec`: transmit power values (in dB) to evaluate.
   * `realiz_num`: number of Monte Carlo iterations.
4. Run `main` from the MATLAB command window.

During execution the script prints a label for each Monte Carlo realization (e.g., `Targets_2_Pt_10_Nr_24_Nt_12_Ratio_0.5__Realiz__37`) to indicate progress. After completion it leaves the following variables in the workspace:

* `P_res`, `P_res_RAB`, `P_res_AA_IS`, `P_res_CSI`: minimum received energy per user for the proposed scheme and the benchmark methods.
* `RMSE_res_G`, `RMSE_res_theta`, `RMSE_res_alpha`, `RMSE_res_LoS`: sensing and channel estimation accuracy metrics.

Use MATLAB plotting utilities to visualize these arrays or save them to disk for post-processing.


## Reproducibility tips

* The scripts set MATLAB's random number generator to the default stream at the start of `main.m`, but additional `rng` calls appear inside some loops. Adjust these seeds if you require fully reproducible runs.
* Simulation time grows with the number of antenna elements, users, and realizations. Begin with small vectors (e.g., `ratio_vec = 0.5`, `realiz_num = 10`) to validate the environment before running the full sweep.

## References

1. O. L. A. López et al., "CSI-Free Rotary Antenna Beamforming for Massive RF Wireless Energy Transfer," IEEE Internet of Things Journal, vol. 9, no. 10, pp. 7375–7387, 2022.
2. O. L. A. López et al., "On CSI-Free Multiantenna Schemes for Massive RF Wireless Energy Transfer," IEEE Internet of Things Journal, vol. 8, no. 1, pp. 278–296, 2021.
3. R. Schmidt, "Multiple emitter location and signal parameter estimation," IEEE Transactions on Antennas and Propagation, vol. 34, no. 3, pp. 276–280, 1986.
4. M. Grant and S. Boyd, "CVX: Matlab software for disciplined convex programming, version 2.1," 2014.

5. ## Citing

If you use this code, please cite the associated article. A BibTeX entry is provided below.

```bibtex
@misc{azarbahram2025sensethenchargewirelesspowertransfer,
      title={Sense-then-Charge: Wireless Power Transfer to Unresponsive Devices with Unknown Location}, 
      author={Amirhossein Azarbahram and Onel L. A. López and Richard D. Souza and Petar Popovski and Matti Latva-aho},
      year={2025},
      eprint={2504.20580},
      archivePrefix={arXiv},
      primaryClass={eess.SP},
      url={https://arxiv.org/abs/2504.20580}, 
}
```

## License

The repository inherits the usage rights granted by the original authors. Refer to the paper or contact the authors for explicit licensing terms.
