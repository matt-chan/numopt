#ifndef NUMOPT_NEWTONDESCENTVECTOR_HPP
#define NUMOPT_NEWTONDESCENTVECTOR_HPP

#include <Eigen/Dense>

#include "common.hpp"



namespace numopt {



class NewtonDescentVector {
private:
    static constexpr size_t MAX_NUMBER_OF_ITERATIONS = 128;
    double convergence_threshold;

    double converged = false;

    const Eigen::VectorXd& x0;  // initial guess
    const VectorFunction f;  // function wrapper for the 'function'
    const Jacobian J;  // function wrapper for the Jacobian


public:
    // CONSTRUCTOR
    /**
     *  Constructor based on an initial guess @param x0, a function wrapper for the function @param f and a function wrapper for the Jacobian @param J.
     */
    NewtonDescentVector(const Eigen::VectorXd& x0, const VectorFunction& f, const Jacobian& J, double convergence_threshold = 1.0e-08);

    // PUBLIC METHODS
    /**
     *  Find and return a minimizer
     */
    Eigen::VectorXd solve();
};



}  // namespace numopt


#endif // NUMOPT_NEWTONDESCENTVECTOR_HPP
