//Estimate P vector, only for full dynamic term

#ifndef PI_VECTOR_HPP_
#define PI_VECTOR_HPP_

#include <Eigen/Dense>
Eigen::Matrix<double, 12, 1> initialize_pi() {
    Eigen::Matrix<double, 12, 1> pi;
    pi << 0.294863538019803,
    0.582153617301607,
    0.475711244170758,
    -1.13599325206214,
    -0.592738925433886,
    2.04405102097999,
    0.104831775807974,
    -3.23927886754168,
    2.38035850706686,
    0.159762113539016,
    0.469079601153411,
    0.240646886069232;
   
    return pi;
}

#endif