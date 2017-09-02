//
//  utils.hpp
//  MPC
//
//  Created by Bachmann, Michael (415) on 02.09.17.
//
//

#ifndef utils_h
#define utils_h

#include <math.h>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

/**
 * polyfit Fit a polynomial.
 * Adapt from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 * @param xvals Eigen::VectorXd with x values of the values to find a fitting polynomial.
 * @param yvals Eigen::VectorXd with y values of the values to find a fitting polynomial.
 * @param order Int value which defines the order of the polynimial to find.
 * @output A vector of Eigen::VectorXd vectors containing the x and y values of the waypoints in vehicle coordinates.
 */
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

/**
 * polyeval Evaluate a polynomial at a specific point.
 * @param coeffs Eigen::VectorXd polynomial coefficients.
 * @param x double value of the point where to evaluate
 * @output double f(x)
 */
double polyeval(Eigen::VectorXd coeffs, double x);

/**
 * polyderivativeeval Calculates the first derivative of a polynomial at a specific point
 * @param coeffs Eigen::VectorXd polynomial coefficients.
 * @param x double value of the point where to evaluate
 * @output double f'(x)
 */
double polyderivativeeval(Eigen::VectorXd coeffs, double x);

/**
 * polyeval Overload: Evaluate a polynomial at a specific point.
 * @param coeffs Eigen::VectorXd polynomial coefficients.
 * @param x CppAD::AD<double> value of the point where to evaluate
 * @output CppAD::AD<double> f(x)
 */
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

/**
 * polyderivativeeval Overload: Calculates the first derivative of a polynomial at a specific point
 * @param coeffs Eigen::VectorXd polynomial coefficients.
 * @param x CppAD::AD<double> value of the point where to evaluate
 * @output CppAD::AD<double> f'(x)
 */
CppAD::AD<double> polyderivativeeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

#endif /* utils_hpp */
