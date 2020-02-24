#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0, 
              0, 1, 0, 0;
  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

  noise_ax = 9;
  noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) 
  {
    MatrixXd P_ = MatrixXd::Identity(4, 4);
    MatrixXd F_ = MatrixXd::Identity(4, 4);
    MatrixXd Q_ = MatrixXd::Identity(4, 4);

    // first measurement
    cout << "EKF: " << endl;
    VectorXd x_(4);
    x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float rho_d = measurement_pack.raw_measurements_[2];

      // Converting radar from polar to cartesian coordinates 
      x_ << rho*cos(theta), rho*sin(theta), rho_d*cos(theta), rho_d*sin(theta); // [x, y, vx, vy]
      ekf_.Init(x_, P_, F_, Hj_, R_radar_, Q_);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      std::cout <<"Before initialization in Laser"  << measurement_pack.sensor_type_ <<std::endl;
      // TODO: Initialize state.
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 1, 1; // [x, y, vx, vy]
      ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_; // updating time stamp information
    is_initialized_ = true;
    return;
  }
  else 
  {
  
    /**
    * Prediction
    */

    float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1.0e6;
    float dt_2 = powf(dt, 2);
    float dt_3 = powf(dt, 3)/2;
    float dt_4 = powf(dt, 4)/4;

    previous_timestamp_ = measurement_pack.timestamp_;  // updating time stamp information

    // State transition matrix F according to the new elapsed time.
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // Process noise covariance matrix.
    ekf_.Q_ (0,0) = dt_4 * noise_ax;
    ekf_.Q_ (0,2) = dt_3 * noise_ax;
    ekf_.Q_ (1,1) = dt_4 * noise_ay;
    ekf_.Q_ (1,3) = dt_3 * noise_ay;
    ekf_.Q_ (2,0) = dt_3 * noise_ax;
    ekf_.Q_ (2,2) = dt_2 * noise_ax;
    ekf_.Q_ (3,1) = dt_3 * noise_ay;
    ekf_.Q_ (3,3) = dt_2 * noise_ay;

    ekf_.Predict();

    /**
    * Update
    */

    /**
    * TODO:
    * - Use the sensor type to perform the update step.
    * - Update the state and covariance matrices.
    */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // Radar updates

      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      // measurement update
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } 
    else 
    {
      // Laser updates

      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      // measurement update
      ekf_.Update(measurement_pack.raw_measurements_);

    }

    // print the output
  } 
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
