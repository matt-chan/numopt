#ifndef NUMOPT_NEWTONDESCENTVECTOR_HPP
#define NUMOPT_NEWTONDESCENTVECTOR_HPP

#include <Eigen/Dense>



namespace numopt {



class NewtonDescentVector {
private:
    static constexpr size_t MAX_NUMBER_OF_ITERATIONS = 128;
    double convergence_threshold;

    double converged = false;

    const Eigen::VectorXd& x0;  // initial guess
    Eigen::VectorXd (*f)(const Eigen::VectorXd& x);  // a pointer to the callable 'function'
    Eigen::MatrixXd (*J)(const Eigen::VectorXd& x);  // a pointer to the callable Jacobian


public:
    // Constructor
    /**
     *  Constructor based on an initial guess @param x0, a (callable) function @param f and a (callable) Jacobian @param J
     */
    NewtonDescentVector(const Eigen::VectorXd& x0, Eigen::VectorXd (*f)(const Eigen::VectorXd& x), Eigen::MatrixXd (*J)(const Eigen::VectorXd& x), double threshold = 1.0e-08);


    // Methods
    /**
     *  Find and return a minimizer
     */
    Eigen::VectorXd solve();

};



}  // namespace numopt


#endif // NUMOPT_NEWTONDESCENTVECTOR_HPP
