#include "DavidsonSolver.hpp"



namespace numopt {


/*
 *  GETTERS
 */
double DavidsonSolver::get_eigenvalue() const {

    if (this->is_solved) {
        return this->eigenvalue;
    } else {
        throw std::runtime_error("You are trying to get an eigenvalue but the solution hasn't been found yet.");
    }
}


Eigen::VectorXd DavidsonSolver::get_eigenvector() const {

    if (this->is_solved) {
        return this->eigenvector;
    } else {
        throw std::runtime_error("You are trying to get an eigenvector but the solution hasn't been found yet.");
    }
}



}  // namespace numopt
