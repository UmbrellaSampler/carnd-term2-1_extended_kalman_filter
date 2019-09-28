#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

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
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                  const std::vector<Eigen::VectorXd> &ground_truth) const;

    /**
     * A helper method to calculate Jacobians.
     */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd &x_state) const;

    /**
     * Converts Cartesian to Polar coordinates
     */
    Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd &x_cart) const;

    /**
     * Converts Polar to Cartesian coordinates
     */
    Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd &x_polar) const;

};

#endif  // TOOLS_H_
