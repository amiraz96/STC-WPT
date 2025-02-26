# STC-WPT
This repository provides the codes for the "Sense-then-charge Protocol for RF Wireless Power Transfer to Unresponsive Devices with Unknown Location" paper.
## CSI-free.m
This file generates the results for the benchmark CSI-free approaches: RAB and AA-IS.
## MUSIC_AoA.m
This file includes the MUSIC algorithm for estimating the AoAs.
## LS_estimate.m
This file performs the least square estimation on the received signal based on the obtained AoAs to estimate the reflection matrix coefficients.
## RF-Beamforming
This file solves the WPT beamforming problem using semi-definite programming and CVX.
## main.m
This file includes an example of a main script, which generates the results for a given setup and plots the figures of RMSE and minimum received power as a function of the sensing allocation coefficient.
