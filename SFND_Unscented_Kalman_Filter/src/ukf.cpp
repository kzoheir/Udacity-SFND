#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

// Debug code to select the best std_a_ and std_yawdd_
#include <fstream> // To get the parameters from a file instead of compilation
#include <string>
// Debug code end

using namespace std;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
// Debug code to select the best std_a_ and std_yawdd_
// Links for a_str: 2.0 seems to be a good value for a_str
//// https://www.google.com/imgres?imgurl=https%3A%2F%2Fqph.fs.quoracdn.net%2Fmain-qimg-228be96e9753eeb5d04c3e8237a18990&imgrefurl=https%3A%2F%2Fwww.quora.com%2FWhy-is-the-maximum-acceleration-at-peak-torque-and-not-at-peak-horsepower-in-a-fixed-gear&docid=UOt1Ibn-sk4veM&tbnid=-L2HbbgnMrv0TM%3A&vet=10ahUKEwjQ66nhvaXjAhVYh1wKHXxODOwQMwhlKBIwEg..i&w=1680&h=1144&client=ubuntu&bih=952&biw=1615&q=car%20accelration%20chart&ved=0ahUKEwjQ66nhvaXjAhVYh1wKHXxODOwQMwhlKBIwEg&iact=mrc&uact=8
//// https://www.google.com/imgres?imgurl=http%3A%2F%2Fspiff.rit.edu%2Fclasses%2Fphys369%2Fworkshops%2Fw1c%2Ftoy_car_position.png&imgrefurl=http%3A%2F%2Fspiff.rit.edu%2Fclasses%2Fphys369%2Fworkshops%2Fw1c%2Fvel_acc.html&docid=DHH_pDNJ35QeCM&tbnid=eb6vrshFheSasM%3A&vet=10ahUKEwiBxNWavqXjAhUJTcAKHXlRA7sQMwhOKAMwAw..i&w=720&h=504&client=ubuntu&bih=952&biw=1615&q=car%20accelration%20time%20curve&ved=0ahUKEwiBxNWavqXjAhUJTcAKHXlRA7sQMwhOKAMwAw&iact=mrc&uact=8
//// https://www.google.com/imgres?imgurl=https%3A%2F%2Fx-engineer.org%2Fwp-content%2Fuploads%2F2018%2F01%2FVehicle-acceleration.jpg%3F41ab8b%2641ab8b&imgrefurl=https%3A%2F%2Fx-engineer.org%2Fprojects%2Fvehicle-acceleration-maximum-speed-modeling-simulation%2F&docid=mSJUsFMFuaEmNM&tbnid=bmWlDNqLsAyg7M%3A&vet=10ahUKEwiBxNWavqXjAhUJTcAKHXlRA7sQMwhMKAEwAQ..i&w=509&h=416&client=ubuntu&bih=952&biw=1615&q=car%20accelration%20time%20curve&ved=0ahUKEwiBxNWavqXjAhUJTcAKHXlRA7sQMwhMKAEwAQ&iact=mrc&uact=8

// Links for std_yawdd_: 25 degress = 0.4 seems to be a good value for std_yawdd_
//// https://www.google.com/imgres?imgurl=https%3A%2F%2Fwww.researchgate.net%2Fprofile%2FMuinuddin_Maharun%2Fpublication%2F290532918%2Ffigure%2Ffig7%2FAS%3A336937618558981%401457343596287%2FVehicle-dynamics-simulation-results-a-Vehicle-yaw-rate-b-Vehicle-path-c-Understeer.png&imgrefurl=https%3A%2F%2Fwww.researchgate.net%2Ffigure%2FVehicle-dynamics-simulation-results-a-Vehicle-yaw-rate-b-Vehicle-path-c-Understeer_fig7_290532918&docid=SXDM7blgCQ808M&tbnid=lDZcxdipdCplcM%3A&vet=10ahUKEwj-gdesv6XjAhW3QkEAHYBqD68QMwhEKAMwAw..i&w=596&h=585&client=ubuntu&bih=952&biw=1615&q=car%20yaw%20rate%20chart&ved=0ahUKEwj-gdesv6XjAhW3QkEAHYBqD68QMwhEKAMwAw&iact=mrc&uact=8

  fstream file;
  std::string a_str, yaw_str;

  file.open("param.txt",ios::in);
  if(!file){  // Use the default values received after testin
    cout<<"Error in opening file..";
    std_a_ = 5.0;
    std_yawdd_ = 0.6;
  }
  else // Use the parameters in: param.txt
  {
    file>>a_str;
    file>>yaw_str;
    std_a_ = std::stod(a_str);
    std_yawdd_ = std::stod(yaw_str);
    cout<<"a_str: "<<a_str<<",yaw_str:"<<yaw_str<<endl;
    cout<<"std_a_: "<<std_a_<<",std_yawdd_:"<<std_yawdd_<<endl;
  }  
// Debug code end

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  //Set state dimension
  n_x_ = 5;
  
 //Set augmented state dimension
  n_aug_ = 7;

  //Sigma spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  
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
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  //Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  //Set weights values
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) =  weight_0;
  for(unsigned int i=1; i < 2*n_aug_+1; i++){
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i)= weight;
  }

  //Sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  //NIS list values
  nis_radar_ = list<double>();
  nis_laser_ = list<double>();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  //Ignore other sensor data if only one sensor is used

  if(!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){
    cout << "Ignore radar sensor" << endl;
    return;
  } else if(!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){

    cout << "Ignore lidar sensor" << endl;
    return;
  } 
  //Initialization
  if (!is_initialized_) {
    cout << "Initialize Filter\n";
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {      
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       * speed can be ignored for the first read 
      */
      double rho    = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      
      //convert to cartesian coordinates
      double px = rho * cos(phi);
      double py = rho * sin(phi);      
      x_ << px, py, 0.0, 0.0, 0.0;
    } else {
      double px = meas_package.raw_measurements_(0);
      double py = meas_package.raw_measurements_(1);
      x_ << px, py, 0.0, 0.0, 0.0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    //store timestamp
    time_us_ = meas_package.timestamp_;

    cout << "Finish first measure\n";
    return;
    }


  // Prediction

  cout << "PREDICT step\n";
  double dt = (meas_package.timestamp_ - time_us_)/ 1000000.0L;
  time_us_ = meas_package.timestamp_;

  //Generate Sigma points and predict state mean and covariance
  Prediction(dt);

  // Update
   
  cout << "UPDATE\n";
  
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar update
    cout << "RADAR UPDATE\n";
    UpdateRadar(meas_package);
  } else {    
    //Lidar update
    cout << "LIDAR UPDATE\n";
    UpdateLidar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.fill(0.0);
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_ +n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ +n_aug_) * L.col(i);
  }

  //predict sigmas
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
	  double yaw_delta=yawd*delta_t;
        px_p = p_x + v/yawd * ( sin (yaw + yaw_delta) - sin_yaw);
        py_p = p_y + v/yawd * ( cos_yaw - cos(yaw+yaw_delta) );
    }
    else {
        px_p = p_x + v*delta_t*cos_yaw;
        py_p = p_y + v*delta_t*sin_yaw;
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    double delta_t2=delta_t*delta_t;
    px_p = px_p + 0.5*nu_a*delta_t2 * cos_yaw;
    py_p = py_p + 0.5*nu_a*delta_t2 * sin_yaw;
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t2;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  //z vector
  const int n_z = 2;
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);

  //Measure
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;

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
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * diff * diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;

  //update 
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_ , n_z );
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = Tc * S_inverse;

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //compute nis
  double nis = z_diff.transpose() * S.inverse() * z_diff;
  nis_laser_.push_back(nis);
  cout << "NIS Lidar: " << nis <<  endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //z vector
  const int n_z = 3;
  VectorXd z = VectorXd(n_z);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);
  z(2) = meas_package.raw_measurements_(2);

  //Measure
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    double sqrt_px_py = sqrt(p_x*p_x + p_y*p_y);

    // measurement model
    Zsig(0,i) = sqrt_px_py;
    Zsig(1,i) = atan2(p_y,p_x);
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt_px_py;
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_ +1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (diff(1)> M_PI) diff(1)-=2.*M_PI;
    while (diff(1)<-M_PI) diff(1)+=2.*M_PI;

    S = S + weights_(i) * diff * diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0,std_radrd_*std_radrd_;
  S = S + R;

  //update 
  //calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_ , n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
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
  MatrixXd S_inverse = S.inverse();
  MatrixXd K = Tc * S_inverse;

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //compute nis
  double nis = z_diff.transpose() * S.inverse() * z_diff;
  nis_radar_.push_back(nis);
  cout << "NIS Radar: " << nis <<  endl;
}
