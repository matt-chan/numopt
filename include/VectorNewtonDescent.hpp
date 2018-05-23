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
#ifndef NUMOPT_VECTORNEWTONDESCENT_HPP
#define NUMOPT_VECTORNEWTONDESCENT_HPP

#include <Eigen/Dense>

#include "common.hpp"
#include "BaseDescent.hpp"



namespace numopt {



class VectorNewtonDescent : public BaseDescent {
private:
    const VectorFunction f;  // function wrapper for the vector 'function'
    const JacobianFunction J;  // function wrapper for the JacobianFunction


public:
    // CONSTRUCTOR
    /**
     *  Constructor based on an initial guess @param x0, a function wrapper for the function @param f and a function wrapper for the Jacobian @param J.
     */
    VectorNewtonDescent(const Eigen::VectorXd& x0, const VectorFunction& f, const JacobianFunction& J, double convergence_threshold = 1.0e-08);


    // OVERRIDDEN PUBLIC METHODS
    /**
     *  Find a solution to the problem f(x) = 0
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member x to the found solution
     */
    void solve() override;
};



}  // namespace numopt


#endif // NUMOPT_VECTORNEWTONDESCENT_HPP
