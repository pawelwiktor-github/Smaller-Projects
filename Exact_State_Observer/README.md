# Exact State Observer

## Table of Contests

1. [General Description](#general-description)
2. [Usage Instructions](#usage-instructions)
3. [File Descriptions](#file-descriptions)
4. [Results](#results)

## General Description
This section is dedicated to the creation and validation under different disturbances of On-Line Exact State Observer (Integral) for "n" order system with sliding window.

## Usage Instructions
1. **Prerequisites**:
   - MATLAB 2024b+ or any tool capable of reading (`.m`, `.mat`, `.slx`) files is required. Additionaly Simulink extension is obligatory

2. **Execution**:
   - Open `exact_state_observer.m` to execute state estimation of "n" order system
   - All particular calculations are performed for observable and controllable of "n" defined order
   - To set forcing and output signals, modify `exact_state_observer_model.slx` for your system purposes
   - The lower the sampling frequency (tau), the more accurate the estimation (for more informations check [ExactStateObserver](/docs/Exact_state_observer.pdf))
   - Data stored in `.mat` files stands for simulation of 2-nd order system with different window length (T) and sampling frequency (tau). It can be loaded and used for printing results.
   - To load the `data.mat` file into MATLAB, use the following command:
   ```matlab
   load('data.mat');

## File Decriptions
### Scripts:
- [exact_state_observer](/main/exact_state_observer.m) parameter definitions, auxilary calculations and state estimation

### Data:
- [data_T_05_tau_1e2](/main/data_T_05_tau_1e2.mat) is data of 2-nd order system estimation for window length T = 0.5 [s] and sampling frequency tau = [1e2]
- [data_T_05_tau_1e3](/main/data_T_05_tau_1e3.mat) is data of 2-nd order system estimation for window length T = 0.5 [s] and sampling frequency tau = [1e3]
- [data_T_05_tau_1e4](/main/data_T_05_tau_1e4.mat) is data of 2-nd order system estimation for window length T = 0.5 [s] and sampling frequency tau = [1e4]
- [data_T_2_tau_1e2](/main/data_T_2_tau_1e3.mat) is data of 2-nd order system estimation for window length T = 2 [s] and sampling frequency tau = [1e3]
- [data_T_2_tau_1e2](/main/data_T_2_tau_1e4.mat) is data of 2-nd order system estimation for window length T = 2 [s] and sampling frequency tau = [1e4]
- [data_T_5_tau_1e2](/main/data_T_5_tau_1e4.mat) is data of 2-nd order system estimation for window length T = 5 [s] and sampling frequency tau = [1e4]

### Model:
- [exact_state_observer_model](/main/exact_state_observer_model.slx) is a prepared simulink model which generates forcing signals (u) and output signal (y) into workspace.

## Results
Results are stored in [Exact_State_Observer](/docs/Exact_state_observer.pdf) file.