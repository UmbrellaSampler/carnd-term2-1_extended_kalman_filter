#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) const {
  /**
   * TODO: Calculate the RMSE here.
   */

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
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

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) const {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 = (c1*c2);

    // check division by zero
    if (fabs(c1) < 0.0001) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd &x_cart) const {
    //h(x) function to convert cartesian coordinates in x_ to polar coordinates
    float px, py, vx, vy;
    px = x_cart(0);
    py = x_cart(1);
    vx = x_cart(2);
    vy = x_cart(3);

    float rho, phi, rho_dot;
    rho = sqrt(px*px + py*py);
    phi = atan2(py, px);

    if (rho < 0.000001){
        //cout << "rho_dot - Error - Division by Zero" << endl;
        rho = 0.000001;
    }
    rho_dot = (px*vx + py*vy) / rho;

    VectorXd x_polar(3);
    x_polar << rho, phi, rho_dot;

    return x_polar;
}

VectorXd Tools::PolarToCartesian(const VectorXd &x_polar) const {
    const auto rho = x_polar(0);
    const auto theta = x_polar(1);
    const auto rho_dot = x_polar(2);
    const auto px = rho * cos(theta);
    const auto py = rho * sin(theta);
    const auto vx = rho_dot * cos(theta);
    const auto vy = rho_dot * sin(theta);

    VectorXd x_cart(4);
    x_cart << px, py, vx, vy;

    return x_cart;
}


