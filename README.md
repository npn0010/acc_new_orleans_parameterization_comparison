# Control Matrix Tuning With Different Parameterizations and Optimization Algorithms
This work was conducted at Auburn University in the ACE Labratory by:  
Nicholas P. Nurre  
Ehsan Taheri  

Please cite:  
TBD  
The accompanying paper, *Comparison of Gain Matrix Parameterizations for Nonlinear Spacecraft Attitude Control*, is currently under review for submission to the 2026 ACC in New Orleans.

## Summary

This repository is the companion software for the paper *Comparison of Gain Matrix Parameterizations for Nonlinear Spacecraft Attitude Control*. This MATLAB code provides a framework to tune one or more square matrices constrained by definiteness, herein referred to as "control matrices," e.g., penalty or gain matrices, for a controller. All that is required is a user-defined function which accepts one or more of these control matrices of arbitrary size and which outputs the meta-cost to be minimized. Four different parameterizations and four different optimization algorithms may be considered. MATLAB's Global Optimization Toolbox (and the Parallel Computing Toolbox if desired to run in parallel) is required . Please feel free to reach out to authors for assistance in using this repository.
