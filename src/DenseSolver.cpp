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
#include "DenseSolver.hpp"

#include <iostream>



namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */

/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem and @param dense_solver_options
 */
DenseSolver::DenseSolver(size_t dim, DenseSolverOptions dense_solver_options) :
    BaseMatrixSolver (dim, dense_solver_options.number_of_requested_eigenpairs),
    matrix (Eigen::MatrixXd::Zero(this->dim, this->dim))
{}

/**
 *   Constructor based on @param matrix of the eigenvalue problem and @param dense_solver_options
 */
DenseSolver::DenseSolver(Eigen::MatrixXd matrix,  DenseSolverOptions dense_solver_options) :
        BaseMatrixSolver (matrix.cols(), dense_solver_options.number_of_requested_eigenpairs),
        matrix (matrix)
{}


/*
 *  PUBLIC OVERRIDDEN METHODS
 */

/**
 *  Solve the full dense eigenvalue problem of @member matrix.
 *
 *  If successful, it sets
 *      - @member is_solved to true
 *      - the number of requested eigenpairs in @member eigenpairs
 */
void DenseSolver::solve() {

    // Solve the dense eigenvalue problem of the Hamiltonian matrix.
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> self_adjoint_eigensolver (this->matrix);  // gives the eigenvalues (and corresponding eigenvectors) in ascending order


    // Set number of requested eigenpairs
    this->_is_solved = true;
    for (size_t i = 0; i < this->number_of_requested_eigenpairs; i++) {
        double eigenvalue = self_adjoint_eigensolver.eigenvalues()(i);
        Eigen::VectorXd eigenvector = self_adjoint_eigensolver.eigenvectors().col(i);

        this->eigenpairs[i] = Eigenpair(eigenvalue, eigenvector);
    }
}


/**
 *  Add @param value to the matrix at (@param index1, @param index2).
 */
void DenseSolver::addToMatrix(double value, size_t index1, size_t index2) {
    this->matrix(index1, index2) += value;
}


}  // namespace eigenproblem
}  // namespace numopt
