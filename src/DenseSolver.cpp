// This file is part of GQCG-numopt.
// 
// Copyright (C) 2017-2018  the GQCG developers
// 
// GQCG-cpputil is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// GQCG-cpputil is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with GQCG-cpputil.  If not, see <http://www.gnu.org/licenses/>.
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
{}



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
    this->matrix(index1, index2) += value;
}


}  // namespace eigenproblem
}  // namespace numopt
