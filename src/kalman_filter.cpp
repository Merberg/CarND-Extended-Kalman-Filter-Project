#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  //create a 4D state vector
  x_ = VectorXd(4);

  //state covariance matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

  //transition matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  //measurement matrix will change with sensor type
  //Laser: H_ = MatrixXd(2, 4)
  //Radar:Hj_ = MatrixXd(3, 4)

  //measurement covariance will change with sensor type
  //Laser: R_ = MatrixXd(2, 2)
  //Radar" R_ = MatrixXd(3, 3)

  //process covariance matrix
  Q_ = MatrixXd(4, 4);
}

KalmanFilter::~KalmanFilter() {
}

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
  x_ = F_ * x_;  // + u = 0
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  CalculateState(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //compute z'
  Eigen::Vector3d z_prev;
  float rho_prev = hypot(x_(0),x_(1));
  z_prev << rho_prev, atan2(x_(1),x_(0)), (x_(0)*x_(2)+x_(1)*x_(3))/rho_prev;
  VectorXd y = z - z_prev;
  CalculateState(y);
}

void KalmanFilter::CalculateState(const Eigen::VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
