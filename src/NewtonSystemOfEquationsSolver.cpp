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
#include "NewtonSystemOfEquationsSolver.hpp"

#include "step.hpp"

#include <iostream>



namespace numopt {
namespace syseq {



/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on an initial guess @param x0, a (callable) function @param f and a (callable) Jacobian @param J
 */
NewtonSystemOfEquationsSolver::NewtonSystemOfEquationsSolver(const Eigen::VectorXd& x0, const VectorFunction& f,
                                                             const JacobianFunction& J, double convergence_threshold) :
    BaseSystemOfEquationsSolver(x0, convergence_threshold),
    f (f),
    J (J)
{}



/*
 *  PUBLIC METHODS
 */

/**
 *  Find a solution to the problem f(x) = 0
 *
 *  If successful, it sets
 *      - @member is_solved to true
 *      - @member x to the found solution
 */
void NewtonSystemOfEquationsSolver::solve() {

    size_t iteration_counter = 0;
    this->x = this->x0;  // start the Newton procedure with the initial guess

    while (!(this->is_solved)) {

        // Calculate the Newton step
        Eigen::VectorXd dx = numopt::newtonStep(this->x, this->f, this->J);

        // Update the current coefficients, using the Newton step
        this->x += dx;


        // Check for convergence
        if (dx.norm() <= this->convergence_threshold) {
            this->is_solved = true;
        } else {  // not is_solved yet
            iteration_counter++;

            // If we reach more than this->maximum_number_of_iterations, the system is considered not to be converging
            if (iteration_counter >= this->maximum_number_of_iterations) {
                throw std::runtime_error("The Newton procedure did not converge.");
            }
        }
    }  // while not is_solved loop
}


}  // namespace syseq
}  // namespace numopt
