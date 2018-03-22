#include "SparseSolver.hpp"

#include <SymEigsSolver.h>
#include <MatOp/SparseSymMatProd.h>



namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */

/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem.
 */
SparseSolver::SparseSolver(size_t dim) :
    BaseMatrixSolver(dim),
    matrix (Eigen::SparseMatrix<double> (this->dim, this->dim))  // Eigen::Sparse is always initiated to zeros
{}



/*
 *  PUBLIC OVERRIDDEN METHODS
 */

/**
 *  Solve the sparse eigenvalue problem of @member matrix.
 *
 *  If successful, it sets
 *      - @member is_solved to true
 *      - @member eigenvalue to the lowest calculated eigenvalue
 *      - @member eigenvector to the associated eigenvector
 */
void SparseSolver::solve() {

    // Solve the Sparse eigenvalue problem of the Hamiltonian matrix.
    Spectra::SparseSymMatProd<double> matrixVectorProduct (this->matrix);
    Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE, Spectra::SparseSymMatProd<double>> spectra_sparse_eigensolver (&matrixVectorProduct, 1, 3);  // request 1 eigenpair, and use 3 Ritz pairs for the solution (need at least 2 more Ritz pairs than requested eigenvalues)
    spectra_sparse_eigensolver.init();
    spectra_sparse_eigensolver.compute();

    // Set the eigenvalue and eigenvector as the lowest-energy eigenpair. We can use index 0 because
    // we have specified Spectra::SMALLEST_ALGE, which selects eigenvalues with smallest algebraic value. Furthermore,
    // we have only requested 1 eigenpair.
    if (spectra_sparse_eigensolver.info() == Spectra::SUCCESSFUL) {
        this->is_solved = true;
        this->eigenvalue = spectra_sparse_eigensolver.eigenvalues()(0);
        this->eigenvector = spectra_sparse_eigensolver.eigenvectors().col(0);
    }
}


/**
 *  Add @param value to the matrix at (@param index1, @param index2).
 */
void SparseSolver::addToMatrix(double value, size_t index1, size_t index2) {
    this->matrix.coeffRef(index1,index2) += value;
}


}  // namespace eigenproblem
}  // namespace numopt
