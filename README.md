# Unscented Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project utilize an Unscented Kalman Filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

[//]: # (Image References)
[image1]: ./images/ProcessNoiseVector.png
[image2]: ./images/ukf_roadmap.png
[image3]: ./images/UKF_Solve.png
[image4]: ./images/UKF_nonlinearSolve.png
[image5]: ./images/sigmapoint.png
[image6]: ./images/RuleForSigmaPointMatrix.png

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.

## Input & Output

* INPUT:
	* ["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)
* OUTPUT:
	* ["estimate_x"] <= kalman filter estimated position x
	* ["estimate_y"] <= kalman filter estimated position y
	* ["rmse_x"]
	* ["rmse_y"]
	* ["rmse_vx"]
	* ["rmse_vy"]

## Unscented Kalman Filter (UKF) Basic

* Extended Kalman Filter (EKF) vs Unscented Kalman Filter (UKF)
	* EKF used a constant velocity (CV) model
		* Due to limitation of CV model, direction will predict turning vehicles incorrectly.
		* The position estimation would tend to result outside of the actually driven circle.
	* UKF used a Constant turn rate and velocity magnitude (CTRV) model
		* ![alt text][image1]

* What is UKF?
	* an alternative technique to deal with norming your process morming your process models or nonlinear measurement models
	* the UKF uses a sigma point to approximate the probability distribution.

* Advantage of UKF.
	* the sigma points approximate the nonlinear transition better than a linearization does.
	* No need to calculate a Jacobian Matrix

* UKF Roadmap

	* ![alt text][image2]
	* Prediction step
		* CTRV model will be used
	* Update step
		* Same top level processing chain with EKF
		
## Sigma Point

* How to solve the nonlinear problem with sigma point?
	* Linear case, we can solve with Kalman filter
		* ![alt text][image3]
		
	* Non-linear case, it isn't easy to solve.
		* ![alt text][image4]
	
	* Using sigma point, it is easy to transform individual sigma point with the non-linear function
		* ![alt text][image5]
		
* How to apply rule for sigma point matrix
	* ![alt text][image6]

# Result

Sample [output](https://github.com/linuxairhead/SDC-C2-P2-Unscented-Kalman-Filter/blob/output/output.txt)

Xsig_aug =
    -7.55494     -7.33225     -7.55494     -7.55494     -7.55494     -7.55494     -7.55494     -7.55494     -7.77762     -7.55494     -7.55494     -7.55494     -7.55494     -7.55494     -7.55494
     10.8436      10.9026      10.9768      10.8436      10.8436      10.8436      10.8436      10.8436      10.7847      10.7104      10.8436      10.8436      10.8436      10.8436      10.8436
     5.29209      7.60516      5.65042       7.9867      5.29209      5.29209      5.29209      5.29209      2.97901      4.93375      2.59747      5.29209      5.29209      5.29209      5.29209
-0.000480931    0.0187473    0.0127577    0.0615576     0.130904 -0.000480931 -0.000480931 -0.000480931   -0.0197092   -0.0137195   -0.0625194    -0.131865 -0.000480931 -0.000480931 -0.000480931
  -0.0656611    -0.435764     -1.13239     0.816704     0.682201      2.69822   -0.0656611   -0.0656611     0.304442      1.00107    -0.948026    -0.813523     -2.82955   -0.0656611   -0.0656611
           0            0            0            0            0            0      51.9615            0            0            0            0            0            0     -51.9615            0
           0            0            0            0            0            0            0      51.9615            0            0            0            0            0            0     -51.9615

Xsig_pred_ =    -7.29033    -6.95201    -7.27249    -7.15697    -7.29323    -7.29112    -7.22538    -7.29033    -7.62869    -7.30829    -7.42556    -7.29341    -7.29122    -7.35528    -7.29033
    10.8431     10.9055     10.9724     10.8763     10.8826     10.8613      10.843     10.8431     10.7829     10.7132     10.8324     10.8035     10.8248     10.8431     10.8431
    5.29209     7.60516     5.65042      7.9867     5.29209     5.29209     7.89016     5.29209     2.97901     4.93375     2.59747     5.29209     5.29209     2.69401     5.29209
-0.00376399 -0.00304087   -0.043862    0.102393    0.165014     0.13443 -0.00376399   0.0611879  -0.0044871    0.036334   -0.109921   -0.172542   -0.141958 -0.00376399  -0.0687159
 -0.0656611   -0.435764    -1.13239    0.816704    0.682201     2.69822  -0.0656611     2.53242    0.304442     1.00107   -0.948026   -0.813523    -2.82955  -0.0656611    -2.66374

42["estimate_marker",{"estimate_x":-7.20593170875779,"estimate_y":10.8532485616518,"rmse_vx":0.849010882489421,"rmse_vy":0.97682518482286,"rmse_x":0.096302366824395,"rmse_y":0.120079326694195}]

|  Performance  |      EKF RMSE     |      UKF RMSE     |
|:-------------:|:-----------------:|:------------------|
|   rmse_vx     |        0.46       |        0.85       |
|   rmse_vy     |        0.45       |        0.98       |
|   rmse_x      |        0.097      |        0.096      |
|   rmse_y      |        0.085      |        0.12       |
