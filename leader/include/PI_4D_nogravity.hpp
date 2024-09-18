//Estimate P vector, only for non-gravity term

#ifndef PI_VECTOR_HPP_
#define PI_VECTOR_HPP_

#include <Eigen/Dense>
Eigen::Matrix<double, 8, 1> initialize_pi() {
    Eigen::Matrix<double, 8, 1> pi;
    pi <<   0.279155659417179,
            0.579341832598150,
            0.482562511076928,
            1.93061407099092,
            2.29278102669192,
            0.274725838566825,
            0.456986110083719,
            0.252131136938608;
   
    return pi;
}

#endif