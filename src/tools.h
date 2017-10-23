#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  VectorXd CalculateRMSE(const vector<VectorXd> &, const vector<VectorXd> &);

  /**
   * A helper method to calculate Jacobians.
   */
  MatrixXd CalculateJacobian(const Eigen::Vector4d&, const MatrixXd&);

};

#endif /* TOOLS_H_ */
