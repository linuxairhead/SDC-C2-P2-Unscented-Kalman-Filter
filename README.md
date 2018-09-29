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


## Noise


	



