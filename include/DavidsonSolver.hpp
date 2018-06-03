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
#ifndef NUMOPT_DAVIDSONSOLVER_HPP
#define NUMOPT_DAVIDSONSOLVER_HPP


#include "BaseEigenproblemSolver.hpp"

#include "common.hpp"



namespace numopt {
namespace eigenproblem {

/**
 *  A class that implements the Davidson algorithm for finding the lowest eigenpair of a (possibly large) diagonally-
 *  dominant symmetric matrix.
 */
class DavidsonSolver : public numopt::eigenproblem::BaseEigenproblemSolver {
private:
    static constexpr size_t maximum_number_of_iterations = 128;

    const double convergence_threshold;  // the tolerance on the norm of the residual vector
    const double correction_threshold;  // the threshold used in solving the (approximated) residue correction equation
    const size_t maximum_subspace_dimension;
    const size_t collapsed_subspace_dimension;

    const numopt::VectorFunction matrixVectorProduct;
    const Eigen::VectorXd diagonal;  // the diagonal of the matrix in question
//    const Eigen::VectorXd t_0;  // the initial guess

    const Eigen::VectorXd V_0;  // the initial guesses



public:
    // CONSTRUCTORS
    /**
     *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal,
     *  and initial guess @param t_0.
     */
//    DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
//                   const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08,
//                   double correction_threshold = 1.0e-12, size_t maximum_subspace_dimension = 15,
//                   size_t collapsed_subspace_dimension = 2);

    /**
     *  Constructor based on a given matrix @param A and an initial guess @param t_0
     */
//    DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08,
//                   double correction_threshold = 1.0e-12, size_t maximum_subspace_dimension = 15,
//                   size_t collapsed_subspace_dimension = 2);


    /**
     *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal,
     *  and a set of initial guesses @param V_0
     */
     DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
                    const Eigen::MatrixXd& V_0, size_t number_of_requested_eigenpairs = 1,
                    double residue_tolerance = 1.0e-08, double correction_threshold = 1.0e-12,
                    size_t maximum_subspace_dimension = 15, size_t collapsed_subspace_dimension);


    // DESTRUCTOR
    ~DavidsonSolver() override = default;


    // GETTERS
    Eigen::VectorXd get_diagonal() override { return this->diagonal; };


    // PUBLIC METHODS
    /**
     *  Solve the eigenvalue problem related to the given matrix-vector product.
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the lowest calculated eigenvalue
     *      - @member eigenvector to the associated eigenvector
     */
    void solve() override;
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_DAVIDSONSOLVER_HPP
