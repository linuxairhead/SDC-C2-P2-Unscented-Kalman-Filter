#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  is_initialized_ = true;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  // State dimension
  n_x_ = x_.size();

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Number of sigma points
  n_sig_ = 2 * n_x_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(n_sig_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::Initialization(MeasurementPackage meas_package) {
  UKF_DEBUG("Initialization","Start");

  float px; // x position
  float py; // y position
  float vx; // x velocity
  float vy; // y velocity
  float v;

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  /**
    * Initialize the state x_ with the first measurement.
    * Create the covariance matrix.///////////////
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
  */
  // first measurement
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UKF_DEBUG("Initialization","RADAR");
    /**
    Convert radar from polar to cartesian coordinates and initialize state.
    */
    float rho = meas_package.raw_measurements_[0];    // radical distance from origin
    float theta = meas_package.raw_measurements_[1];  // angle between ro and x axis
    float rho_dot = meas_package.raw_measurements_[2];// radical velocity 

    px = rho * cos(theta);
    py = rho * sin(theta);
    vx = rho_dot * cos(theta);
    vy = rho_dot * sin(theta);
    v = sqrt( vx*vx + vy*vy);

  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UKF_DEBUG("Initialization","LASER");
    /**
    Initialize state.
    */
    px = meas_package.raw_measurements_[0]; // x position
    py = meas_package.raw_measurements_[1]; // y position
    v = 0; // since Laser cannot measure velocity, set velocity as zero
  }

  // initialize x state.
  x_ << px, py, v, 0, 0;  // x, y, vx, vy
   
  // Initialize weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++)
    weights_(i) = 0.5 / (n_aug_ + lambda_);

  time_us_ = meas_package.timestamp_;

  // done initializing, no need to predict or update
  is_initialized_ = true;
  return;
}
  
  
  /**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	
  if (!is_initialized_) {
    UKF_DEBUG("ProcessMeasurement","initialization");
    Initialization(meas_package);
    return;
  }

  UKF_DEBUG("ProcessMeasurement","prediction");
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  UKF_DEBUG("ProcessMeasurement", dt);
  Prediction(dt);


  UKF_DEBUG("ProcessMeasurement","update");
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {

    UKF_DEBUG("ProcessMeasurement","update laser");
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

    UKF_DEBUG("ProcessMeasurement","update radar");
    UpdateRadar(meas_package);
   }

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  UKF_DEBUG("Prediction","Start");
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  UKF_DEBUG("UpdateLidar","Start");
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  UKF_DEBUG("UpdateRadar","Start");
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
