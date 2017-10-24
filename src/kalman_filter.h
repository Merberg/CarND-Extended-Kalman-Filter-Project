#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
 public:

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Query the Kalman Filter state
   */
  Eigen::Vector4d GetState() {
    return x_;
  }

  /**
   * Init Initializes Kalman filter
   * @param px initial
   * @param py initial
   * @param timestamp initial
   */
  void Init(float, float, long long);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param timestamp
   */
  void Predict(long long);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param H measurement matrix
   * @param R measurement covariance
   * @param the measurement at k+1
   */
  void Update(const Eigen::MatrixXd &, const Eigen::MatrixXd &,
              const Eigen::VectorXd &);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param Hj measurement matrix
   * @param R measurement covariance
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::MatrixXd &, const Eigen::MatrixXd &,
                 const Eigen::VectorXd &);

 private:

  // state vector
  Eigen::Vector4d x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // previous timestamp
  long long previous_timestamp_;

  /**
   * Updates the state, given y, using the Kalman Filter equations
   * @param H measurement matrix
   * @param R measurement covariance
   * @param y calculated
   */
  void SetState(const Eigen::MatrixXd &, const Eigen::MatrixXd &,
                const Eigen::VectorXd &);

};

#endif /* KALMAN_FILTER_H_ */
