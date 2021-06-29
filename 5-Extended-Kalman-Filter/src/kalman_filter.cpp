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
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  
  //compute the error y
  MatrixXd y = z - H_ * x_;

  // Compute Kalman gain
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
    
  // Update estimate
  x_ = x_ + K * y;
  
  // I Matrix
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  
  // calculate P'
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  

  // get the predicted states into measurement space
  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px);
  double rho__ = std::max(rho, 0.0001);
  double rho_dot = (px * vx + py * vy) / rho__ ;

  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  VectorXd y = z - z_pred;

  // Normalize angle
  //std::cout<< "y angle before: " << y(1)<<'\n';
  while (y(1) > M_PI) y(1) -= 2 * M_PI;
  while (y(1) < -M_PI) y(1) += 2 * M_PI;
  //std::cout<< "angle after: " << y(1)<<'\n';
  
  // Compute Kalman gain
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
    
  // Update estimate
  x_ = x_ + K * y;
  
  // I Matrix
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  
  // calculate P'
  P_ = (I - K * H_) * P_;
}
