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
 *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal, and
 *  initial guess @param t_0.
 */
DavidsonSolver::DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
                               const Eigen::VectorXd& t_0, double residue_tolerance, double correction_threshold,
                               size_t maximum_subspace_dimension, size_t collapsed_subspace_dimension) :
    BaseEigenproblemSolver(static_cast<size_t>(t_0.size())),
    matrixVectorProduct (matrixVectorProduct),
    diagonal (diagonal),
    t_0 (t_0),
    convergence_threshold (residue_tolerance),
    correction_threshold (correction_threshold),
    maximum_subspace_dimension (maximum_subspace_dimension),
    collapsed_subspace_dimension (collapsed_subspace_dimension)
{
    if (maximum_subspace_dimension <= collapsed_subspace_dimension) {
        throw std::invalid_argument("maximum_subspace_dimension should be at least larger than collapsed_subspace_dimension");
    }
}


/**
 *  Constructor based on a given matrix @param A and an initial guess @param t_0
 */
DavidsonSolver::DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& t_0, double residue_tolerance,
                               double correction_threshold, size_t maximum_subspace_dimension,
                               size_t collapsed_subspace_dimension) :
    DavidsonSolver([A](const Eigen::VectorXd& x) { return A * x; },  // lambda matrix-vector product function created from the given matrix A
                   A.diagonal(), t_0, residue_tolerance, correction_threshold, maximum_subspace_dimension,
                   collapsed_subspace_dimension)
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

    // Calculate a new subspace vector
    Eigen::VectorXd v_1 = this->t_0 / this->t_0.norm();


    // Calculate the expensive matrix-vector product
    Eigen::VectorXd vA_1 = this->matrixVectorProduct(v_1);


    // Expand the subspaces V and VA
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(this->dim, 1);
    V.col(0) = v_1;

    Eigen::MatrixXd VA = Eigen::MatrixXd::Zero(this->dim, 1);
    VA.col(0) = vA_1;


    // Calculate the Rayleigh quotient, i.e. the subspace matrix of size 1
    double theta = v_1.dot(vA_1);
    Eigen::MatrixXd M = Eigen::MatrixXd::Constant(1, 1, theta);


    // Calculate the associated residual vector
    Eigen::VectorXd r = vA_1 - theta * v_1;

    // Check for convergence
    if (r.norm() < this->convergence_threshold) {
        this->is_solved = true;
        this->eigenvalue = theta;
        this->eigenvector = v_1;
    }


    size_t iteration_counter = 1;
    while (!(this->is_solved)) {

        // Approximately solve the residue equation by using coefficient-wise quotients
        // This implementation was adapted from Klaas' DOCI code: (https://github.com/klgunst/doci)
        Eigen::VectorXd denominator = this->diagonal - Eigen::VectorXd::Constant(this->dim, theta);
        Eigen::VectorXd t = (denominator.array().abs() > this->correction_threshold).select(r.array() / denominator.array().abs(),
                                                                                            r / this->correction_threshold);


        // Project on the orthogonal subspace of V
        Eigen::VectorXd t_orthogonal = t - V * (V.transpose() * t);


        // Calculate the new subspace vector
        Eigen::VectorXd v = t_orthogonal / t_orthogonal.norm();


        // Calculate the expensive matrix-vector product
        Eigen::VectorXd vA = this->matrixVectorProduct(v);


        // If needed, collapse the subspace to this->collapsed_subspace_dimension 'best' eigenvectors
        if (V.cols() == this->maximum_subspace_dimension) {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver (M);
            theta = eigensolver.eigenvalues()(0);
            Eigen::VectorXd s = eigensolver.eigenvectors().col(0);

            Eigen::MatrixXd lowest_eigenvectors = eigensolver.eigenvectors().topLeftCorner(this->maximum_subspace_dimension,
                                                                                           this->collapsed_subspace_dimension);

            V = V * lowest_eigenvectors;
            VA = VA * lowest_eigenvectors;

            // The subspace matrix should now again be a (this->collapsed_subspace_dimension)x(this->collapsed_subspace_dimension)-matrix
            M = V.transpose() * VA;
        }


        // Expand the subspaces V and VA
        V.conservativeResize(Eigen::NoChange, V.cols()+1);
        V.col(V.cols()-1) = v;

        VA.conservativeResize(Eigen::NoChange, VA.cols()+1);
        VA.col(VA.cols()-1) = vA;


        // Calculate the subspace matrix
        M.conservativeResize(M.rows()+1, M.cols()+1);

        Eigen::VectorXd m_k = V.transpose() * vA;
        M.col(M.cols()-1) = m_k;
        M.row(M.rows()-1) = m_k;


        // Solve the subspace eigenvalue problem
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver (M);
        theta = eigensolver.eigenvalues()(0);
        Eigen::VectorXd s = eigensolver.eigenvectors().col(0);


        // Calculate the new residual vector
        Eigen::VectorXd u = V * s;
        Eigen::VectorXd uA = VA * s;

        r = uA - theta * u;


        // Check for convergence
        if (r.norm() < this->convergence_threshold) {
            this->is_solved = true;
            this->eigenvalue = theta;
            this->eigenvector = u;
        }

        else {  // not converged yet
            iteration_counter ++;

            // If we reach more than this->maximum_number_of_iterations, the system is considered not to be converging
            if (iteration_counter >= this->maximum_number_of_iterations) {
                throw std::runtime_error("The Davidson algorithm did not converge.");
            }
        }

    }  // while loop
}


}  // namespace eigenproblem
}  // namespace numopt
