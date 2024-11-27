//Calculated Y matrix, w/ gravity term.
#ifndef Y_MATRIX_HPP_
#define Y_MATRIX_HPP_

#include <iostream>
#include <math.h>
#include <Eigen/Dense>

// Function to calculate the Y matrix
Eigen::Matrix<double,2 ,8> calculate_Y_matrix(const Eigen::Vector4d theta, Eigen::Vector4d thetad, Eigen::Vector4d thetadd) {
    
    Eigen::Matrix<double, 2, 8> Y;
    Y.setZero(); // Initialize the matrix with zeros
    // thetad[1] = thetad[1]*tanh(coeff*abs(thetad[1]));
    // thetad[3] = thetad[3]*tanh(coeff*abs(thetad[3]));
    // thetadd[1] = thetadd[1]*tanh(coeff*abs(thetadd[1]));
    // thetadd[3] = thetadd[3]*tanh(coeff*abs(thetadd[3]));
    // Using 0-based indexing: theta[1] is theta2, theta[3] is theta4, and similarly for thetad and thetadd
    Y(0, 0) = 2.0 * cos(theta[3]) * thetad[3] * thetad[1] + cos(theta[3]) * pow(thetad[3], 2) + 2.0 * sin(theta[3]) * thetadd[1] + sin(theta[3]) * thetadd[3];
    Y(0, 1) = 2.0 * cos(theta[3]) * thetadd[1] - sin(theta[3]) * pow(thetad[3], 2) - 2.0 * sin(theta[3]) * thetad[3] * thetad[1] + cos(theta[3]) * thetadd[3];
    Y(0, 2) = thetadd[3];
    Y(0, 3) = thetadd[1];
    Y(0, 4) = tanh(20 * thetad[1]);
    Y(0, 5) = thetad[1];
    Y(0, 6) = 0.0;
    Y(0, 7) = 0.0;

    Y(1, 0) = sin(theta[3]) * thetadd[1] - cos(theta[3]) * pow(thetad[1], 2);
    Y(1, 1) = sin(theta[3]) * pow(thetad[1], 2) + cos(theta[3]) * thetadd[1];
    Y(1, 2) = thetadd[1] + thetadd[3];
    Y(1, 3) = 0.0;
    Y(1, 4) = 0.0;
    Y(1, 5) = 0.0;
    Y(1, 6) = tanh(20 * thetad[3]);
    Y(1, 7) = thetad[3];

    return Y;
}

#endif // Y_MATRIX_HPP_
