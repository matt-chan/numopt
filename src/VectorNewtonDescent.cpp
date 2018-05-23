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
#include "VectorNewtonDescent.hpp"

#include <iostream>



namespace numopt {



/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on an initial guess @param x0, a (callable) function @param f and a (callable) Jacobian @param J
 */
VectorNewtonDescent::VectorNewtonDescent(const Eigen::VectorXd& x0, const VectorFunction& f, const JacobianFunction& J, double convergence_threshold) :
    x0 (x0),
    f (f),
    J (J),
    convergence_threshold (convergence_threshold)
{}



/*
 *  PUBLIC METHODS
 */

/**
 *  Find and return a minimizer
 */
Eigen::VectorXd VectorNewtonDescent::solve() {

    size_t iteration_counter = 0;


    Eigen::VectorXd x = this->x0;  // start the Newton procedure with the initial guess
    while (!(this->converged)) {

        // 1. Calculate f(x) and J(x)
        Eigen::VectorXd f = this->f(x);
        Eigen::MatrixXd J = this->J(x);

        // 2. Solve the Newton step
        Eigen::VectorXd dx = J.colPivHouseholderQr().solve(-f);

        // 3. Update the geminal coefficients
        x += dx;

        // 4. Check for convergence
        if (dx.norm() <= this->convergence_threshold) {
            this->converged = true;
            return x;
        }

        else {  // not is_solved yet
            iteration_counter ++;

            // If we reach more than this->MAX_NUMBER_OF_ITERATIONS, the system is considered not to be converging
            if (iteration_counter >= this->MAX_NUMBER_OF_ITERATIONS) {
                throw std::runtime_error("The Newton procedure did not converge.");
            }
        }
    }  // while not is_solved loop
}



}  // namespace numopt
