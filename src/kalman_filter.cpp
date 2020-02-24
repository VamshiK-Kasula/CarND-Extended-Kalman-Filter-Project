#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
  std::cout <<"Initialized" <<std::endl;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
  x_ = F_ * x_;
  MatrixXd F_trans = F_.transpose();

  P_ = F_ * P_ * F_trans + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd h = H_ * x_;
	VectorXd y = z - h;

  UpdateEstimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) 
{
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px*px + py*py);

  if (fabs(rho) < 0.000001) 
  {
      rho =  0.001;
  }
  float phi = atan2(py, px);
  float rho_d = (px * vx + py * vy)/rho;

  VectorXd h(3);
  h << rho, phi, rho_d;
	VectorXd y = z - h;

  if (fabs(y(1)) > 2 * M_PI) 
  {
    if (phi < 0)
    {
      phi += 2 * M_PI;
    }
    else 
    {
      phi -= 2 * M_PI;
    }
    h(1) = phi;
    y(1) = z(1) - h(1);
	}

  UpdateEstimate(y);
}


void KalmanFilter::UpdateEstimate(const Eigen::VectorXd &y)
{
  MatrixXd Ht_trans = H_.transpose();
	MatrixXd S = H_ * P_ * Ht_trans + R_;
	MatrixXd S_inv = S.inverse();
	MatrixXd K = P_ * Ht_trans * S_inv;

	// new estimate
	x_ = x_ + (K * y);
	P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}