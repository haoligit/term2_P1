# **Project 1 Submission in CarND-Term2**
This repository contains project 1 submission in Self-Driving Car Nano-Degree, Term 2, Extended Kalman Filters. Modified source codes are all included in folder ./src. The following files are modified:

##### 1. FusionEKF.cpp
##### 2. kalman_filter.cpp
##### 3. kalman_fliter.h
##### 4. tools.cpp

Besides the standard EKF predict and update functions, a few special treatments are implemented.

1. When initialize `ekf.x_`, for both RADAR and LASER measurement, only `X` and `Y` are initialized accordingly, and `VX` and `VY` are set to `ZERO`, since speed couldn't be determined by single measurement.

2. When initialize `ekf.P_`, covariance of `VX` and `VY` are set to a very large value (1000). Covariance of `X` and `Y` are set to `0.02` for LASER measurement, and set to `1` for RADAR measurement, since LASER gives more accurate position measurement than RADAR.

3. For RADAR measurement update, after using `atan2()` to calculate predicted direction, the angle difference between measurement and prediction is wrapped around to stay within -PI to PI. 

When running with the simulator, the following tracking RMSE are reported:

|    RMSE   | X   |  Y | VX | VY |
|:---------:|:---:|:--:|:--:|:--:|
| Data set 1 | 0.0966 | 0.0851 | 0.4477 | 0.4217|
| Data set 2 | 0.0726 | 0.0967 | 0.4582 | 0.4971|
