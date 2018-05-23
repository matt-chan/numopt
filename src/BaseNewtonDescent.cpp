#include "BaseNewtonDescent.hpp"



namespace numopt {


/*
 *  CONSTRUCTORS
 */
/**
 *  Constructor based on a given initial guess @param x0 and a @param convergence_threshold
 */
BaseNewtonDescent::BaseNewtonDescent(const Eigen::VectorXd& x0, double convergence_threshold) :
    x0 (x0),
    convergence_threshold (convergence_threshold)
{}



/*
 *  GETTERS
 */
Eigen::VectorXd BaseNewtonDescent::get_solution() const {
    if (!this->is_solved) {
        throw std::logic_error("The solution hasn't been found and you are trying to get it.");
    } else {
        return this->x;
    }
}



}  // namespace numopt
