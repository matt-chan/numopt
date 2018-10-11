#include <utility>

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
#ifndef NUMOPT_EIGENPROBLEMSOLVEROPTIONS_HPP
#define NUMOPT_EIGENPROBLEMSOLVEROPTIONS_HPP


#include <cstddef>
#include <Eigen/Dense>


namespace numopt {
namespace eigenproblem {


/**
 *  An enum class for the implemented eigenproblem solver types
 */
enum class SolverType {
    DENSE,
    SPARSE,
    DAVIDSON
};


/**
 *  POD (plain object data) struct to specify eigenproblem solver options
 *  The derived structs can be used to interface with the eigenproblem solvers implemented in numopt::eigenproblem
 */
struct BaseSolverOptions {
public:
    virtual numopt::eigenproblem::SolverType get_solver_type() = 0;

    /*
     *  MEMBERS
     */
    size_t number_of_requested_eigenpairs = 1;
};



/**
 *  POD struct to specify dense eigenproblem solver options
 */
struct DenseSolverOptions : public BaseSolverOptions {
public:
    /*
     *  MEMBERS
     */
    numopt::eigenproblem::SolverType solver_type = numopt::eigenproblem::SolverType::DENSE;

    /*
     *  METHODS
     */
    numopt::eigenproblem::SolverType get_solver_type () override { return this->solver_type; };
};


/**
 *  POD struct to specify sparse eigenproblem solver options
 */
struct SparseSolverOptions : public BaseSolverOptions {
public:
    /*
     *  MEMBERS
     */
    numopt::eigenproblem::SolverType solver_type = numopt::eigenproblem::SolverType::SPARSE;

    /*
     *  METHODS
     */
    numopt::eigenproblem::SolverType get_solver_type () override { return this->solver_type; };
};


/**
 *  POD struct to specify Davidson eigenproblem solver options
 */
struct DavidsonSolverOptions : public BaseSolverOptions {
public:
    /*
     *  MEMBERS
     */
    numopt::eigenproblem::SolverType solver_type = numopt::eigenproblem::SolverType::DAVIDSON;

    double residue_tolerance = 1.0e-08;  // the tolerance on the norm of the residual vector
    double correction_threshold = 1.0e-12;  // the threshold used in solving the (approximated) residue correction equation

    size_t maximum_subspace_dimension = 15;
    size_t collapsed_subspace_dimension = 2;

    Eigen::MatrixXd X_0;  // Eigen::MatrixXd of initial guesses, or Eigen::VectorXd of initial guess

    /*
     *  CONSTRUCTOR
     */
    explicit DavidsonSolverOptions(Eigen::MatrixXd initial_guess) :
        X_0(std::move(initial_guess))
    {}

    /*
     *  METHODS
     */
    numopt::eigenproblem::SolverType get_solver_type () override { return this->solver_type; };
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_EIGENPROBLEMSOLVEROPTIONS_HPP
