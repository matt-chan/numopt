#include "DenseSolver.hpp"

#include <iostream>



namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */

/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem.
 */
DenseSolver::DenseSolver(size_t dim) :
    BaseMatrixSolver(dim),
    matrix (Eigen::MatrixXd::Zero(this->dim, this->dim))
{
    std::cout << "Made a dense solver with dimension " << dim << std::endl;
}



/*
 *  PUBLIC OVERRIDDEN METHODS
 */

/**
 *  Solve the full dense eigenvalue problem of @member matrix.
 */
void DenseSolver::solve() {

    // Solve the dense eigenvalue problem of the Hamiltonian matrix.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> self_adjoint_eigensolver (this->matrix);


    // Set the eigenvalue and eigenvector as the lowest-energy eigenpair. We can use index 0 because
    // SelfAdjointEigenSolver gives the eigenvalues (and corresponding eigenvalues) in ascending order.
    this->is_solved = true;
    this->eigenvalue = self_adjoint_eigensolver.eigenvalues()(0);
    this->eigenvector = self_adjoint_eigensolver.eigenvectors().col(0);
}


/**
 *  Add @param value to the matrix at (@param index1, @param index2).
 */
void DenseSolver::addToMatrix(double value, size_t index1, size_t index2) {

    std::cout << "Adding to dense matrix: "
              << "value " << value
              << "\tindex1:" << index1
              << "\tindex2:" << index2 << std::endl;
    this->matrix(index1, index2) += value;
}


}  // namespace eigenproblem
}  // namespace numopt
