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

  is_initialized_ = false;

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
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  // Initialize weights
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < weights_.size(); i++)
    weights_(i) = 0.5 / (n_aug_ + lambda_);
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

  // init covariance matrix
  P_ << 0.15,    0, 0, 0, 0,
               0, 0.15, 0, 0, 0,
               0,    0, 1, 0, 0,
               0,    0, 0, 1, 0,
               0,    0, 0, 0, 1;  
  
  // set current time 
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

  UKF_DEBUG("ProcessMeasurement","Start");

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
 * Generate augmented sigma point
 * @param {MatrixXd} Xsig_out
 */
MatrixXd UKF::AugmentedSigmaPoints() {

  UKF_DEBUG("AugmentedSigmaPoints","Start");

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;
  
  return Xsig_aug;
}

/**
 * Sigma point prediction
 */
void UKF::SigmaPointPrediction(MatrixXd* Xsig_aug, double delta_t) {

  UKF_DEBUG("SigmaPointPrediction","Start");

  MatrixXd Xsig_aug_temp =  *Xsig_aug;
  
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug_temp(0,i);
    double p_y = Xsig_aug_temp(1,i);
    double v = Xsig_aug_temp(2,i);
    double yaw = Xsig_aug_temp(3,i);
    double yawd = Xsig_aug_temp(4,i);
    double nu_a = Xsig_aug_temp(5,i);
    double nu_yawdd = Xsig_aug_temp(6,i);

    std::cout << "p_x = " << p_x << std::endl;
    std::cout << "p_y = " << p_y << std::endl;
    std::cout << "v = " << v << std::endl;
    std::cout << "yaw = " << yaw << std::endl;
    std::cout << "yawd = " << yawd << std::endl;
    std::cout << "nu_a = " << nu_a << std::endl;
    std::cout << "nu_yawdd = " << nu_yawdd << std::endl;
    
    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {        
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    std::cout << "px_p = " << px_p << std::endl;
    std::cout << "py_p = "<< py_p << std::endl;
    
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    std::cout << "yaw_p = " << yaw_p << std::endl;
    std::cout << "yawd_p = "<< yawd_p << std::endl;
    
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  //print result
  std::cout << "Xsig_pred_ = " << Xsig_pred_ << std::endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  UKF_DEBUG("Prediction","Start");

  /**
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  //create sigma point matrix
  MatrixXd Xsig_aug =  AugmentedSigmaPoints();
  UKF_DEBUG("Prediction","AugmentedSigmaPoints Done");
  
  //predict sigma point
  SigmaPointPrediction(&Xsig_aug, delta_t);
  UKF_DEBUG("Prediction","SigmaPointPrediction Done");

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
    std::cout << "x_ = " << x_ << std::endl;
    std::cout << "weights_ = " <<  weights_(i)  << std::endl;
    std::cout << "Xsig_pred_.col = " << Xsig_pred_.col(i) << std::endl;
    std::cout << "count " << i << std::endl;
  }
  UKF_DEBUG("Prediction","predicted state mean Done");
  
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
  //for (int i = 0; i < 7; i++) {  //iterate over sigma points  

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    std::cout << "x_diff(3) = " << x_diff(3) << std::endl;    
    std::cout << "x_ = " << x_ << std::endl;
    std::cout << "weights_ = " <<  weights_(i)  << std::endl;
    std::cout << "Xsig_pred_.col = " << Xsig_pred_.col(i) << std::endl;
    std::cout << "before while loop " << i << std::endl;    
    //angle normalization
    while (x_diff(3)> M_PI) { 
      x_diff(3)-=2.*M_PI;
      std::cout << i << " first count while loop " <<  x_diff(3) << std::endl;    
    }
 
    while (x_diff(3)<-M_PI) {
      x_diff(3)+=2.*M_PI;
      std::cout << "second while loop " << i << std::endl;   
    }
    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    std::cout << "count " << i << std::endl;    
  }
  UKF_DEBUG("Prediction","state covariance matrix Done");
  //print result
  std::cout << "x_ = " << std::endl << x_ << std::endl;
  std::cout << "P_ = " << std::endl << P_ << std::endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  UKF_DEBUG("UpdateLidar","Start");
  /**
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
    //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, lidar can measure p_x and p_y
  int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = p_x;
    Zsig(1, i) = p_y;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;
  
  //print result
  std::cout << "S = " << std::endl << S << std::endl;

  // UKF Update for Ladar
  UpdateState(n_z, z_pred, z, Zsig, S);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  UKF_DEBUG("UpdateRadar","Start");
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  //extract measurement as VectorXd
  VectorXd z = meas_package.raw_measurements_;

  //set measurement dimension, lidar can measure p_x and p_y
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //print result
  std::cout << "S = " << std::endl << S << std::endl;

  // UKF Update for Radar
  UpdateState(n_z, z_pred, z, Zsig, S);
}

/**
 * Updates the state and the state covariance matrix
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateState(int n_z, VectorXd z_pred, VectorXd z, MatrixXd Zsig, MatrixXd S) {

  UKF_DEBUG("UpdateState","Start");

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //print result
  std::cout << "Updated state x: " << std::endl << x_ << std::endl;
  std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;
}
