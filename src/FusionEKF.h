#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
   * Query the Kalman Filter state
   */
  Eigen::Vector4d GetState() {
    return ekf_.GetState();
  }

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &);

private:
  /**
  * Kalman Filter class is housed here
  */
  KalmanFilter ekf_;

  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;

  /**
   * Process the Measurements for Lidar and Radar (splitting into methods to
   * enable change to Template Method in the event another sensor type is added
   * later.
   */
  void ProcessLidarMeasurement(const MeasurementPackage &);
  void ProcessRadarMeasurement(const MeasurementPackage &);
};

#endif /* FusionEKF_H_ */
