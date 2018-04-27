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
#include "BaseMatrixSolver.hpp"



namespace numopt {
namespace eigenproblem {



// CONSTRUCTOR
/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem.
 */
BaseMatrixSolver::BaseMatrixSolver(size_t dim) :
    BaseEigenproblemSolver(dim)
{}



}  // namespace eigenproblem
}  // namespace numopt
