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
#ifndef NUMOPT_NEWTONDESCENTVECTOR_HPP
#define NUMOPT_NEWTONDESCENTVECTOR_HPP

#include <Eigen/Dense>

#include "common.hpp"



namespace numopt {



class NewtonDescentVector {
private:
    static constexpr size_t MAX_NUMBER_OF_ITERATIONS = 128;
    double convergence_threshold;

    double converged = false;

    const Eigen::VectorXd& x0;  // initial guess
    const VectorFunction f;  // function wrapper for the 'function'
    const Jacobian J;  // function wrapper for the Jacobian


public:
    // CONSTRUCTOR
    /**
     *  Constructor based on an initial guess @param x0, a function wrapper for the function @param f and a function wrapper for the Jacobian @param J.
     */
    NewtonDescentVector(const Eigen::VectorXd& x0, const VectorFunction& f, const Jacobian& J, double convergence_threshold = 1.0e-08);

    // PUBLIC METHODS
    /**
     *  Find and return a minimizer
     */
    Eigen::VectorXd solve();
};



}  // namespace numopt


#endif // NUMOPT_NEWTONDESCENTVECTOR_HPP
