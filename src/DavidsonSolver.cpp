#include "DavidsonSolver.hpp"



namespace numopt {


/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on a given matrix-vector product function @param matrixVectorProduct initial guess @param t_0.
 */
DavidsonSolver::DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08, double correction_threshold = 1.0e-03, size_t maximum_subspace_dimension = 15) :
    matrixVectorProduct (matrixVectorProduct),
    t_0 (t_0),
    residue_tolerance (residue_tolerance),
    correction_threshold (correction_threshold),
    maximum_subspace_dimension (maximum_numer_of_iterations)
{}



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
