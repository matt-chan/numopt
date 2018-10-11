// This file is part of GQCG-numopt.
// 
// Copyright (C) 2017-2018  the GQCG developers
// 
// GQCG-numopt is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// GQCG-numopt is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with GQCG-numopt.  If not, see <http://www.gnu.org/licenses/>.
#include "SparseSolver.hpp"

#include <Spectra/SymEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>



namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */

/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem and @param sparse_solver_options
 */
SparseSolver::SparseSolver(size_t dim, SparseSolverOptions sparse_solver_options) :
    BaseMatrixSolver (dim, sparse_solver_options.number_of_requested_eigenpairs),
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
 *      - the number of requested eigenpairs in @member eigenpairs
 */
void SparseSolver::solve() {

    // Solve the sparse eigenvalue problem of the Hamiltonian matrix
    Spectra::SparseSymMatProd<double> matrixVectorProduct (this->matrix);

    // Request the number of eigenpairs, and use at least 2 more Ritz pairs than requested eigenvalues)
    Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE, Spectra::SparseSymMatProd<double>> spectra_sparse_eigensolver (&matrixVectorProduct, static_cast<int>(this->number_of_requested_eigenpairs), static_cast<int>(this->number_of_requested_eigenpairs) + 2);
    spectra_sparse_eigensolver.init();
    spectra_sparse_eigensolver.compute();

    // Set number of requested eigenpairs

    // Set the eigenvalue and eigenvector as the lowest-energy eigenpair. We can use increasing indices because
    // we have specified Spectra::SMALLEST_ALGE, which selects eigenvalues with smallest algebraic value
    if (spectra_sparse_eigensolver.info() == Spectra::SUCCESSFUL) {
        this->_is_solved = true;

        for (size_t i = 0; i < this->number_of_requested_eigenpairs; i++) {
            double eigenvalue = spectra_sparse_eigensolver.eigenvalues()(i);
            Eigen::VectorXd eigenvector = spectra_sparse_eigensolver.eigenvectors().col(i);

            this->eigenpairs[i] = Eigenpair(eigenvalue, eigenvector);
        }
    } else {  // if Spectra was not successful
        throw std::runtime_error("Spectra could not solve the sparse eigenvalue problem.");
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
