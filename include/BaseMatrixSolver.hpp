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
#ifndef NUMOPT_BASEMATRIXSOLVER_HPP
#define NUMOPT_BASEMATRIXSOLVER_HPP



#include "BaseEigenproblemSolver.hpp"



namespace numopt {
namespace eigenproblem {


class BaseMatrixSolver : public numopt::eigenproblem::BaseEigenproblemSolver {
public:
    // CONSTRUCTOR
    /**
     *   Constructor based on the dimension @param dim of the eigenvalue problem.
     */
    explicit BaseMatrixSolver(size_t dim);


    // DESTRUCTOR
    ~BaseMatrixSolver() override = default;


    // PUBLIC PURE VIRTUAL METHODS
//    virtual Eigen::MatrixXd get_diagonal() = 0;

    /**
     *  Add @param value to the matrix at (@param index1, @param index2).
     */
    virtual void addToMatrix(double value, size_t index1, size_t index2) = 0;
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_BASEMATRIXSOLVER_HPP
