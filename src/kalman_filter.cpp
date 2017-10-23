#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter()
    : previous_timestamp_(0) {
  //create a 4D state vector
  x_ = Eigen::Vector4d();

  //state covariance matrix
  P_ = MatrixXd(4, 4);
  P_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 1000;

  //transition matrix
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;

  //measurement matrix will change with sensor type and provided by parameter
  //Laser: H_ = MatrixXd(2, 4)
  //Radar:Hj_ = MatrixXd(3, 4)

  //measurement covariance will change with sensor type and provided by parameter
  //Laser: R_ = MatrixXd(2, 2)
  //Radar" R_ = MatrixXd(3, 3)

  //process covariance matrix
  Q_ = MatrixXd(4, 4);
}

KalmanFilter::~KalmanFilter() {
}

//******************************************************************************
void KalmanFilter::Init(float px, float py, long long ts) {
  static const float EKF_INIT_VX = 0;
  static const float EKF_INIT_VY = 0;

  /**
   Initialize state.
   */
  x_ = VectorXd(4);
  x_(0) = px;
  x_(1) = py;
  x_(2) = EKF_INIT_VX;
  x_(3) = EKF_INIT_VY;

  // record the time
  previous_timestamp_ = ts;
}

//******************************************************************************
void KalmanFilter::Predict(long long ts) {
  static const float NOISE_AX = 9.0;
  static const float NOISE_AY = 9.0;

  float dt = (ts - previous_timestamp_) / 1000000.0;  //in seconds
  previous_timestamp_ = ts;
  float dt_2 = dt * dt;
  float dt_3 = dt * dt_2;
  float dt_4 = dt * dt_3;

  // incorporate the time in the transition matrix
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // update the process noise covariance matrix
  Q_ << dt_4 / 4 * NOISE_AX, 0, dt_3 / 2 * NOISE_AX, 0, 0, dt_4 / 4 * NOISE_AY, 0, dt_3
      / 2 * NOISE_AY, dt_3 / 2 * NOISE_AX, 0, dt_2 * NOISE_AX, 0, 0, dt_3 / 2
      * NOISE_AY, 0, dt_2 * NOISE_AY;

  x_ = F_ * x_;  // + u = 0
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

//******************************************************************************
void KalmanFilter::Update(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                          const Eigen::VectorXd &z) {
  VectorXd y = z - H * x_;
  SetState(H, R, y);
}

//******************************************************************************
void KalmanFilter::UpdateEKF(const Eigen::MatrixXd &Hj,
                             const Eigen::MatrixXd &R,
                             const Eigen::VectorXd &z) {
  //compute z'
  Eigen::Vector3d z_prev;
  float ro = hypot(x_(0), x_(1));
  float theta = atan2(x_(1), x_(0));
  //guard against divide by zero with reusing the old
  float ro_dot = (ro == 0) ? z[2] : (x_(0) * x_(2) + x_(1) * x_(3)) / ro;

  z_prev << ro, theta, ro_dot;
  VectorXd y = z - z_prev;

  //normalize theta
  y[1] = atan2(sin(y[1]), cos(y[1]));

  SetState(Hj, R, y);
}

//******************************************************************************
void KalmanFilter::SetState(const Eigen::MatrixXd &H, const Eigen::MatrixXd &R,
                            const Eigen::VectorXd &y) {
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H) * P_;
}
