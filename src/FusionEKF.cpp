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

  //
  if (measurements.sensor_type_ == MeasurementPackage::LASER) {
    ProcessLidarMeasurement(measurements);
  } else {
    ProcessRadarMeasurement(measurements);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}

//******************************************************************************
void FusionEKF::ProcessLidarMeasurement(
    const MeasurementPackage &measurements) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    float px = measurements.raw_measurements_(0);
    float py = measurements.raw_measurements_(1);
    ekf_.Init(px, py, measurements.timestamp_);

    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  ekf_.Predict(measurements.timestamp_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  ekf_.Update(H_laser_, R_laser_, measurements.raw_measurements_);

}

//******************************************************************************
void FusionEKF::ProcessRadarMeasurement(
    const MeasurementPackage &measurements) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    float ro = measurements.raw_measurements_(0);
    float theta = measurements.raw_measurements_(1);
    ekf_.Init(ro * cos(theta), ro * sin(theta), measurements.timestamp_);

    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  ekf_.Predict(measurements.timestamp_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  Hj_ = tools.CalculateJacobian(ekf_.GetState(), Hj_);
  ekf_.UpdateEKF(Hj_, R_radar_, measurements.raw_measurements_);
}
