//Calculated Y matrix, w/ gravity term.
#ifndef Y_MATRIX_HPP_
#define Y_MATRIX_HPP_

#include <math.h>
#include <Eigen/Dense>

// Function to calculate the Y matrix
Eigen::Matrix<double,2 ,8> calculate_Y_matrix(const Eigen::Vector4d theta, Eigen::Vector4d thetad, Eigen::Vector4d thetadd, const double coeff) {
    
    Eigen::Matrix<double, 2, 8> Y;
    Y.setZero(); // Initialize the matrix with zeros
    double g = 9.81;
    // double coeff = 1000;
    // thetad[0] = thetad[0]*tanh(coeff*abs(thetad[0]));
    // thetad[2] = thetad[2]*tanh(coeff*abs(thetad[2]));
    // thetadd[0] = thetadd[0]*tanh(coeff*abs(thetadd[0]));
    // thetadd[2] = thetadd[2]*tanh(coeff*abs(thetadd[2]));
    // Using 0-based indexing: theta[1] is theta2, theta[3] is theta4, and similarly for thetad and thetadd
    Y(0, 0) = 2.0 * cos(theta[2]) * thetad[2] * thetad[0] + cos(theta[2]) * pow(thetad[2], 2) + 2.0 * sin(theta[2]) * thetadd[0] + sin(theta[2]) * thetadd[2];
    Y(0, 1) = 2.0 * cos(theta[2]) * thetadd[0] - sin(theta[2]) * pow(thetad[2], 2) - 2.0 * sin(theta[2]) * thetad[2] * thetad[0] + cos(theta[2]) * thetadd[2];
    Y(0, 2) = thetadd[2];
    Y(0, 3) = thetadd[0];
    Y(0, 4) = tanh(20 * thetad[0]);
    Y(0, 5) = thetad[0];
    Y(0, 6) = 0.0;
    Y(0, 7) = 0.0;

    Y(1, 0) = sin(theta[2]) * thetadd[0] - cos(theta[2]) * pow(thetad[0], 2);
    Y(1, 1) = sin(theta[2]) * pow(thetad[0], 2) + cos(theta[2]) * thetadd[0];
    Y(1, 2) = thetadd[0] + thetadd[2];
    Y(1, 3) = 0.0;
    Y(1, 4) = 0.0;
    Y(1, 5) = 0.0;
    Y(1, 6) = tanh(20 * thetad[2]);
    Y(1, 7) = thetad[2];

    return Y;
}

#endif // Y_MATRIX_HPP_
