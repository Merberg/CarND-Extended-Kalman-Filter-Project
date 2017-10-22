#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {
}

//******************************************************************************
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurements) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    if (measurements.sensor_type_ == MeasurementPackage::LASER) {
      float px = measurements.raw_measurements_(0);
      float py = measurements.raw_measurements_(1);
      initialize(px, py, measurements.timestamp_);
    } else {
      float ro = measurements.raw_measurements_(0);
      float theta = measurements.raw_measurements_(1);
      initialize(ro * cos(theta), ro * sin(theta), measurements.timestamp_);
    }
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  predict(measurements.timestamp_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurements.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    ekf_.Update(measurements.raw_measurements_);
  } else {
    Hj_ = tools.CalculateJacobian(ekf_.x_, Hj_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurements.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

//******************************************************************************
void FusionEKF::initialize(float x, float y, long long ts) {
  //TODO play for better RMSE
  static const float EKF_INIT_VX = 1;
  static const float EKF_INIT_VY = 0;

  /**
   Initialize state.
   */
  ekf_.x_ = VectorXd(4);
  ekf_.x_(0) = x;
  ekf_.x_(1) = y;
  ekf_.x_(2) = EKF_INIT_VX;
  ekf_.x_(3) = EKF_INIT_VY;

  // record the time
  previous_timestamp_ = ts;

  // done initializing, no need to predict or update
  is_initialized_ = true;
}

//******************************************************************************
void FusionEKF::predict(long long ts) {
  static const float NOISE_AX = 9.0;
  static const float NOISE_AY = 9.0;

  float dt = (ts - previous_timestamp_) / 1000000.0;  //in seconds
  previous_timestamp_ = ts;
  float dt_2 = dt * dt;
  float dt_3 = dt * dt_2;
  float dt_4 = dt * dt_3;

  // incorporate the time in the transition matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update the process noise covariance matrix
  ekf_.Q_ << dt_4 / 4 * NOISE_AX, 0, dt_3 / 2 * NOISE_AX, 0, 0, dt_4 / 4
      * NOISE_AY, 0, dt_3 / 2 * NOISE_AY, dt_3 / 2 * NOISE_AX, 0, dt_2
      * NOISE_AX, 0, 0, dt_3 / 2 * NOISE_AY, 0, dt_2 * NOISE_AY;

  ekf_.Predict();
}
