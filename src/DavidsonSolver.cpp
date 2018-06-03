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
#include "DavidsonSolver.hpp"
#include <iostream>



#include <chrono>



namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal,
 *  and a set of initial guesses @param V_0
 */
DavidsonSolver::DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
                               const Eigen::MatrixXd& V_0, size_t number_of_requested_eigenpairs,
                               double residue_tolerance, double correction_threshold,
                               size_t maximum_subspace_dimension, size_t collapsed_subspace_dimension) :
   BaseEigenproblemSolver(static_cast<size_t>(V_0.rows()), number_of_requested_eigenpairs),
   matrixVectorProduct (matrixVectorProduct),
   diagonal (diagonal),
   V_0 (V_0),
   convergence_threshold (residue_tolerance),
   correction_threshold (correction_threshold),
   maximum_subspace_dimension (maximum_subspace_dimension),
   collapsed_subspace_dimension (collapsed_subspace_dimension)
{
    if (V_0.cols() < this->number_of_requested_eigenpairs) {
        throw std::invalid_argument("You have to specify at least as many initial guesses as number of requested eigenpairs.");
    }

    if (this->collapsed_subspace_dimension < this->number_of_requested_eigenpairs) {
        throw::std::invalid_argument("The collapsed subspace dimension must be at least the number of requested eigenpairs.");
    }
}


/**
 *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal, and
 *  one initial guess @param x_0
 */
//DavidsonSolver::DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
//                               const Eigen::VectorXd& x_0, size_t number_of_requested_eigenpairs, double residue_tolerance,
//                               double correction_threshold, size_t maximum_subspace_dimension,
//                               size_t collapsed_subspace_dimension) :
//    DavidsonSolver(matrixVectorProduct, diagonal, Eigen::Map<const Eigen::MatrixXd>(x_0.data(), x_0.size(), 1),
//                   number_of_requested_eigenpairs, residue_tolerance, correction_threshold, maximum_subspace_dimension,
//                   collapsed_subspace_dimension)
//{}


/**
 *  Constructor based on a given matrix @param A and an initial guess @param x_0
 */
DavidsonSolver::DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& x_0, size_t number_of_requested_eigenpairs,
                               double residue_tolerance, double correction_threshold, size_t maximum_subspace_dimension,
                               size_t collapsed_subspace_dimension) :
    DavidsonSolver([A](const Eigen::VectorXd& x) { return A * x; }, // lambda matrix-vector product function created from the given matrix A
                   A.diagonal(), x_0, number_of_requested_eigenpairs, residue_tolerance, correction_threshold,
                   maximum_subspace_dimension, collapsed_subspace_dimension)
{}


/*
 *  PUBLIC METHODS
 */

/**
 *  Solve the eigenvalue problem related to the given matrix-vector product.
 *
 *  If successful, sets
 *      - @member is_solved to true
 *      - @member eigenvalue to the calculated eigenvalue
 *      - @member eigenvector to the calculated eigenvector
 */
void DavidsonSolver::solve() {

    // Calculate the initial subspace matrix S
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(this->V_0.cols(), this->V_0.cols());
    Eigen::MatrixXd VA = Eigen::MatrixXd::Zero(this->V_0.cols(), this->V_0.cols());  // stores already calculated matrix-vector products Av

    // Calculate the necessary matrix-vector products
    for (size_t j = 0; j < this->V_0.cols(); j++) {
        Eigen::VectorXd v_j = this->V_0.col(j);
        Eigen::VectorXd vA_j = this->matrixVectorProduct(v_j);
        VA.col(j) = vA_j;
    }

    Eigen::MatrixXd V = this->V_0;
    S = V.transpose() * VA;


    size_t iteration_counter = 0;
    while (!(this->is_solved)) {

        // Calculate the subspace matrix
        // After an iteration, we are dimension_difference = V.cols() - S.cols() short in dimension of S
        auto dimension_difference = static_cast<size_t>(V.cols() - S.cols());
        S.conservativeResize(S.rows()+dimension_difference, S.cols()+dimension_difference);

        for (size_t j = static_cast<size_t>(S.cols()); j < V.cols(); j++) {
            Eigen::VectorXd v_j = V.col(j);
            Eigen::VectorXd vA_j = this->matrixVectorProduct(v_j);
            VA.col(j) = vA_j;

            Eigen::VectorXd s_j = V.transpose() * vA_j;  // s_j = V^T vA_j
            S.col(j) = s_j;
            S.row(j) = s_j;
        }


        // Diagonalize the subspace matrix and find the r (this->number_of_requested_eigenpairs) lowest eigenpairs
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver (S);
        Eigen::VectorXd Lambda = eigensolver.eigenvalues().head(this->number_of_requested_eigenpairs);
        Eigen::MatrixXd Z = eigensolver.eigenvectors().topLeftCorner(this->dim, this->number_of_requested_eigenpairs);


        // Calculate current estimates
        Eigen::MatrixXd X = V * Z;


        // Calculate the residual vectors
        Eigen::MatrixXd R = VA * Z - Lambda.asDiagonal() * X;


        Eigen::MatrixXd Delta = Eigen::MatrixXd::Zero(this->dim, this->number_of_requested_eigenpairs);
        for (size_t column_index = 0; column_index < R.cols(); column_index++) {

            // Solve the residual equations
            Eigen::VectorXd denominator = this->diagonal - Eigen::VectorXd::Constant(this->dim, Lambda(column_index));
            Delta.col(column_index) = (denominator.array().abs() > this->correction_threshold).select(R.col(column_index).array() / denominator.array().abs(),
                                                                                                      R.col(column_index) / this->correction_threshold);
            Delta.col(column_index).normalize();


            // Project the correction vectors on the orthogonal complement of V
            Eigen::VectorXd v = Delta.col(column_index) - V * (V.transpose() * Delta.col(column_index));
            double norm = v.norm();  // calculate the norm before normalizing
            v.normalize();

            if (norm > 1.0e-03) {  // include in the new subspace
                V.conservativeResize(Eigen::NoChange, V.cols()+1);
                V.col(V.cols()-1) = v;

                Eigen::VectorXd vA = this->matrixVectorProduct(v);
                VA.conservativeResize(Eigen::NoChange, VA.cols()+1);
                VA.col(VA.cols()-1) = vA;
            }
        }


        // Check for convergence
        if (!((Delta.colwise().norm().array() > this->convergence_threshold).any())) {  // CLion can give errors that .any() is not found, but it compiles
            this->is_solved = true;

            for (size_t i = 0; i < this->number_of_requested_eigenpairs; i++) {
                double eigenvalue = Lambda(i);
                Eigen::VectorXd eigenvector = X.col(i);

                this->eigenpairs[i] = numopt::eigenproblem::Eigenpair(eigenvalue, eigenvector);
            }
        } else {
            iteration_counter++;

            // If we reach more than this->maximum_number_of_iterations, the system is considered not to be converging
            if (iteration_counter >= this->maximum_number_of_iterations) {
                throw std::runtime_error("The Davidson algorithm did not converge.");
            }
        }


        // Do a subspace collapse if necessary
        if (V.cols() > this->maximum_subspace_dimension) {
            Eigen::MatrixXd lowest_eigenvectors = eigensolver.eigenvectors().topLeftCorner(this->dim, this->collapsed_subspace_dimension);

            V = V * lowest_eigenvectors;
            VA = VA * lowest_eigenvectors;
            S = V.transpose() * VA;
        }
    }
}


}  // namespace eigenproblem
}  // namespace numopt
