#include "NewtonDescentVector.hpp"



namespace numopt {



/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on an initial guess @param x0, a (callable) function @param f and a (callable) Jacobian @param J
 */
NewtonDescentVector::NewtonDescentVector(const Eigen::VectorXd& x0, Eigen::VectorXd (*f)(Eigen::VectorXd& x), Eigen::MatrixXd (*J)(Eigen::VectorXd& x), double threshold) :
    x0 (x0),
    f (f),
    J (J),
    convergence_threshold (threshold)
{}



/*
 *  PUBLIC METHODS
 */

/**
 *  Find and return a minimizer
 */
Eigen::VectorXd NewtonDescentVector::solve() {

    size_t iteration_counter = 0;


    Eigen::VectorXd x = this->x0;  // start the Newton procedure with the initial guess
    while ((!this->converged)) {

        // 1. Calculate F(x) and J(x)
        Eigen::VectorXd F = (*this->f)(x);
        Eigen::MatrixXd J = (*this->J)(x);

        // 2. Solve the Newton step
        Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-F);

        // 3. Update the geminal coefficients
        x += dx;

        // 4. Check for convergence
        if (dx.norm() <= this->convergence_threshold) {
            this->converged = true;
            return x;
        }

        else {  // not converged yet
            iteration_counter ++;

            // If we reach more than this->MAX_NUMBER_OF_ITERATIONS, the system is considered not to be converging
            if (iteration_counter >= this->MAX_NUMBER_OF_ITERATIONS) {
                throw std::runtime_error("The Newton procedure did not converge.");
            }
        }
    }  // while not converged loop
}



}  // namespace numopt
