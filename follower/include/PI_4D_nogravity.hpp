//Estimate P vector, only for non-gravity term

#ifndef PI_VECTOR_HPP_
#define PI_VECTOR_HPP_

#include <Eigen/Dense>
Eigen::Matrix<double, 8, 1> initialize_pi() {
    Eigen::Matrix<double, 8, 1> pi;
    pi <<   0.2566,
    0.1713,
    0.0239,
    1.3975,
    2.3685,
    0.4797,
    0.6683,
    0.6900;



    return pi;
}

#endif