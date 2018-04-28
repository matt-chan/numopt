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
#ifndef NUMOPT_DENSESOLVER_HPP
#define NUMOPT_DENSESOLVER_HPP



#include "BaseMatrixSolver.hpp"



namespace numopt {
namespace eigenproblem {


class DenseSolver : public numopt::eigenproblem::BaseMatrixSolver {
private:
    Eigen::MatrixXd matrix;


public:
    // CONSTRUCTOR
    /**
     *   Constructor based on the dimension @param dim of the eigenvalue problem.
     */
    explicit DenseSolver(size_t dim);


    // DESTRUCTOR
    ~DenseSolver() override = default;


    // GETTERS
    Eigen::MatrixXd get_matrix() { return this->matrix; };


    // PUBLIC OVERRIDDEN METHODS
    /**
     *  Solve the full dense eigenvalue problem of @member matrix.
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the lowest calculated eigenvalue
     *      - @member eigenvector to the associated eigenvector
     */
    void solve() override;

    /**
     *  Add @param value to the matrix at (@param index1, @param index2).
     */
    void addToMatrix(double value, size_t index1, size_t index2) override;


    // GETTERS
    Eigen::MatrixXd get_matrix(){return this->matrix;};

};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_DENSESOLVER_HPP
