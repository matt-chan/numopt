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
