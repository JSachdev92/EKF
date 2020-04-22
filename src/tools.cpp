#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  // recover state parameters
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 
  float px2 = px*px;
  float py2 = py*py;
  float px2py2 = px2+py2;
  float sqrtpxpy = sqrt(px2py2);
  float pxpy32 = pow(px2py2,(3/2));
  // check division by zero
  if (fabs(px2py2) < 0.0001) {
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
  }
  else{
      Hj << (px/sqrtpxpy), (py/sqrtpxpy), 0, 0,
            -(py/px2py2), (px/px2py2), 0, 0,
            (py*(vx*py-vy*px)/pxpy32), (px*(vy*px-vx*py)/pxpy32), (px/sqrtpxpy), (py/sqrtpxpy);
  }

  return Hj;
}
