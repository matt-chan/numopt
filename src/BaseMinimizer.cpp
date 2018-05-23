#include "BaseMinimizer.hpp"



namespace numopt {
namespace minimization {


/*
 *  CONSTRUCTORS
 */
/**
 *  Constructor based on a given initial guess @param x0 and a @param convergence_threshold
 */
BaseMinimizer::BaseMinimizer(const Eigen::VectorXd& x0, double convergence_threshold) :
    x0(x0),
    convergence_threshold(convergence_threshold) {}


/*
 *  GETTERS
 */
Eigen::VectorXd BaseMinimizer::get_solution() const {
    if (!this->is_solved) {
        throw std::logic_error("The solution hasn't been found and you are trying to get it.");
    } else {
        return this->x;
    }
}


}  // namespace minimization
}  // namespace numopt
