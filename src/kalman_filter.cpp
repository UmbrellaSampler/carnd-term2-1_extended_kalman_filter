#include "kalman_filter.h"
#include "tools.h"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

const MatrixXd I = MatrixXd::Identity(4, 4);
// const VectorXd u = VectorXd::Zero(2);

const Tools tools{};


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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
    /**
     * TODO: predict the state
     */
    x_ = F_ * x_;  // + u;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
     * TODO: update the state by using Kalman Filter equations
     */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     * TODO: update the state by using Extended Kalman Filter equations
     */

    const auto h = tools.CartesianToPolar(x_);
    VectorXd y = z - h;

    // normalize the phi between -pi and pi
    while(y[1] > M_PI){
        y[1] -= 2*M_PI;
    }
    while(y[1] < -M_PI){
        y[1] += 2*M_PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    P_ = (I - K * H_) * P_;
}

std::ostream &operator<<(std::ostream &os, const KalmanFilter &filter) {
    os << "x_:\n" << filter.x_ << "\nP_:\n" << filter.P_ << "\nF_:\n" << filter.F_ << "\nQ_:\n" << filter.Q_ << "\nH_:\n"
       << filter.H_ << "\nR_:\n" << filter.R_;
    return os;
}
