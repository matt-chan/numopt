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
#ifndef NUMOPT_BASEEIGENVALUESOLVER_HPP
#define NUMOPT_BASEEIGENVALUESOLVER_HPP



#include <cstddef>
#include <Eigen/Dense>



namespace numopt {
namespace eigenproblem {


class BaseEigenproblemSolver {
protected:
    const size_t dim;  // the dimension of the vector space associated to the eigenvalue problem

    bool is_solved = false;
    double eigenvalue;
    Eigen::VectorXd eigenvector;


    // PROTECTED CONSTRUCTORS
    /**
     *  Protected constructor to initialize the const @member dim by @param dim.
     */
    explicit BaseEigenproblemSolver(size_t dim);


public:
    // DESTRUCTOR
    virtual ~BaseEigenproblemSolver() = default;


    // GETTERS
    virtual Eigen::VectorXd get_diagonal() = 0;

    double get_eigenvalue() const;

    Eigen::VectorXd get_eigenvector() const;
    double get_eigenvector(size_t index) const;


    // PUBLIC PURE VIRTUAL METHODS
    /**
     *  Solve the eigenvalue problem associated to the eigenproblem solver.
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the lowest calculated eigenvalue
     *      - @member eigenvector to the associated eigenvector
     */
    virtual void solve() = 0;
};


}  // namespace eigenproblem
}  // namespace numopt





#endif  // NUMOPT_BASEEIGENVALUESOLVER_HPP
