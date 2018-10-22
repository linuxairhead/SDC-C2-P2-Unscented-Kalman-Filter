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
    -7.51601     -7.39085     -7.51601     -7.51601     -7.51601     -7.51601     -7.51601     -7.51601     -7.64116     -7.51601     -7.51601     -7.51601     -7.51601 -7.51601     -7.51601
      10.884      10.9206      10.9994       10.884       10.884       10.884       10.884       10.884      10.8475      10.7687       10.884       10.884       10.884   10.884       10.884
     5.09054      5.27762      5.11935      5.28268      5.09054      5.09054      5.09054      5.09054      4.90346      5.06172       4.8984      5.09054      5.09054  5.09054      5.09054
  -0.0189229 -5.18917e-05    0.0242339  -0.00784044    0.0379085   -0.0189229   -0.0189229   -0.0189229    -0.037794   -0.0620798   -0.0300054   -0.0757544   -0.0189229   -0.0189229   -0.0189229
   -0.051093   -0.0284945  -0.00830314   -0.0267382     0.120801    0.0902192    -0.051093    -0.051093   -0.0736914   -0.0938828   -0.0754477    -0.222987    -0.192405-0.051093    -0.051093
           0            0            0            0            0            0      1.55885            0            0            0            0            0            0 -1.55885            0
           0            0            0            0            0            0            0      1.55885            0            0            0            0            0        0     -1.55885
Xsig_pred_ =    -7.26153    -7.12697    -7.26011    -7.25188    -7.26169    -7.26152    -7.25958    -7.26153    -7.39618    -7.26345    -7.27121    -7.26232    -7.26155-7.26348    -7.26153
    10.8789     10.9204     11.0055     10.8818     10.8945     10.8798     10.8789     10.8789     10.8377     10.7524     10.8762     10.8634      10.878     10.878910.8789
    5.09054     5.27762     5.11935     5.28268     5.09054     5.09054     5.16848     5.09054     4.90346     5.06172      4.8984     5.09054     5.09054      5.01265.09054
 -0.0214776 -0.00147662   0.0238188 -0.00917735   0.0439486   -0.014412  -0.0214776   -0.019529  -0.0414785  -0.0667739  -0.0337778  -0.0869037  -0.0285432  -0.0214776  -0.0234261
  -0.051093  -0.0284945 -0.00830314  -0.0267382    0.120801   0.0902192   -0.051093   0.0268493  -0.0736914  -0.0938828  -0.0754477   -0.222987   -0.192405   -0.051093   -0.129035

42["estimate_marker",{"estimate_x":-7.24319825244068,"estimate_y":10.8714745677027,"rmse_vx":0.328088486482503,"rmse_vy":0.226492455674689,"rmse_x":0.0637626413063266,"rmse_y":0.0826642183659209}]

|  Performance  |      EKF RMSE     |      UKF RMSE     |
|:-------------:|:-----------------:|:------------------|
|   rmse_vx     |        0.46       |        0.32       |
|   rmse_vy     |        0.45       |        0.22       |
|   rmse_x      |        0.097      |        0.064      |
|   rmse_y      |        0.085      |        0.082     |
