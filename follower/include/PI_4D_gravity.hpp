//Estimate P vector, only for non-gravity term

#ifndef PI_VECTOR_HPP_
#define PI_VECTOR_HPP_

#include <Eigen/Dense>
Eigen::Matrix<double, 8, 1> initialize_pi() {
    Eigen::Matrix<double, 8, 1> pi;
    pi <<   0.175649849915331,
            0.184907885729660,
            -0.00610134716492867,
            1.48556843509966,
            2.31271725934920,
            0.481696527313974,
            0.690746943507402,
            0.499834003241676;

    return pi;
}

#endif