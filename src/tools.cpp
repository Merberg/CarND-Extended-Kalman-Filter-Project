#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
}

Tools::~Tools() {
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;
  unsigned int e_size = estimations.size();

  //Parameter checking
  if (e_size == 0 || e_size != ground_truth.size()) {
    cout << "Tools::CalculateRMSE - parameter error" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for (unsigned int i = 0; i < e_size; ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    //coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / e_size;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  if (px == 0 || py == 0 || vx == 0 || vy == 0) {
    cout << "Tools::CalculateJacobian - divide by zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  float p_ss = px * px + py * py;
  float p_hypot = sqrt(p_ss);
  float p_1p5 = pow(p_ss, 3 / 2);
  float dif = vx * py - vy * px;

  Hj << px / p_hypot, py / p_hypot, 0, 0, -py / p_ss, px / p_ss, 0, 0, py * dif
      / p_1p5, px * dif / p_1p5, px / p_hypot, py / p_hypot;

  return Hj;
}
