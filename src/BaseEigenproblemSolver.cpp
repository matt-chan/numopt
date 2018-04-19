#include "BaseEigenproblemSolver.hpp"
#include <iostream>


namespace numopt {
namespace eigenproblem {


/*
 *  PROTECTED CONSTRUCTORS
 */

/**
 *  Protected constructor to initialize the const @member dim by @param dim.
 */
BaseEigenproblemSolver::BaseEigenproblemSolver(size_t dim) :
    dim (dim),
    eigenvalue (0.0),
    eigenvector (Eigen::VectorXd::Zero(this->dim))
{}



/*
 *  GETTERS
 */

double BaseEigenproblemSolver::get_eigenvalue() const {

    if (this->is_solved) {
        return this->eigenvalue;
    } else {
        throw std::logic_error("The eigenvalue problem hasn't been solved yet and you are trying to get the eigenvalue.");
    }
}

Eigen::VectorXd BaseEigenproblemSolver::get_eigenvector() const {

    if (this->is_solved) {
        return this->eigenvector;
    } else {
        throw std::logic_error("The eigenvalue problem hasn't been solved yet and you are trying to get the eigenvector.");
    }
}


double BaseEigenproblemSolver::get_eigenvector(size_t index) const {

    if (this->is_solved) {
        return this->eigenvector(index);
    } else {
        throw std::logic_error("The eigenvalue problem hasn't been solved yet and you are trying to get the eigenvector.");
    }
}



}  // namespace eigenproblem
}  // namespace numopt
