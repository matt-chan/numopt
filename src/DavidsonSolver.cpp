#include "DavidsonSolver.hpp"
#include <iostream>



namespace numopt {


/*
 *  CONSTRUCTORS
 */

/**
 *  Constructor based on a given matrix-vector product function @param matrixVectorProduct initial guess @param t_0.
 */
DavidsonSolver::DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal, const Eigen::VectorXd& t_0, double residue_tolerance, double correction_threshold, size_t maximum_subspace_dimension) :
    t_0 (t_0),
    matrixVectorProduct (matrixVectorProduct),
    diagonal (diagonal),
    dim (static_cast<size_t>(this->t_0.size())),
    residue_tolerance (residue_tolerance),
    correction_threshold (correction_threshold),
    maximum_subspace_dimension (maximum_subspace_dimension)
{}


/**
 *  Constructor based on a given matrix @param A and an initial guess @param t_0
 */
DavidsonSolver::DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& t_0, double residue_tolerance, double correction_threshold, size_t maximum_subspace_dimension) :
    t_0 (t_0),
    matrixVectorProduct ([A](const Eigen::VectorXd& x) { return A * x; }),  // lambda matrix-vector product function created from the given matrix A
    diagonal (A.diagonal()),
    dim (static_cast<size_t>(this->t_0.size())),
    residue_tolerance (residue_tolerance),
    correction_threshold (correction_threshold),
    maximum_subspace_dimension (maximum_subspace_dimension)
{}



/*
 *  GETTERS
 */

double DavidsonSolver::get_eigenvalue() const {

    if (this->is_solved) {
        return this->eigenvalue;
    } else {
        throw std::runtime_error("You are trying to get an eigenvalue but the solution hasn't been found yet.");
    }
}


Eigen::VectorXd DavidsonSolver::get_eigenvector() const {

    if (this->is_solved) {
        return this->eigenvector;
    } else {
        throw std::runtime_error("You are trying to get an eigenvector but the solution hasn't been found yet.");
    }
}



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
double DavidsonSolver::solve() {

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
    if (r.norm() < this->residue_tolerance) {
        this->is_solved = true;
        this->eigenvalue = theta;
        this->eigenvector = v_1;
    }


    size_t iteration_counter = 1;
    while (!(this->is_solved)) {
        // Approximately solve the residue equation by using coefficient-wise quotients
        Eigen::VectorXd denominator = Eigen::VectorXd::Constant(this->dim, theta) - this->diagonal;
        Eigen::VectorXd t = (denominator.array().abs() >= correction_threshold).select(r.cwiseQuotient(denominator), Eigen::VectorXd::Zero(this->dim));


        // Project on the orthogonal subspace of V
        Eigen::VectorXd t_orthogonal = t - V * (V.transpose() * t);


        // Calculate the new subspace vector
        Eigen::VectorXd v = t_orthogonal / t_orthogonal.norm();


        // Calculate the expensive matrix-vector product
        Eigen::VectorXd vA = this->matrixVectorProduct(v);


        // If needed, collapse the subspace to 2 dimensions
        //      The final collapsed subspace should be spanned by the guess vector from the previous iteration and
        //      the one calculated in this iteration.
        if (V.cols() == this->maximum_subspace_dimension) {

            // Eigen's conservativeResize only keeps the values in the top-left corner, so swap the last and the first
            // column and afterwards to the resize
            V.col(0).swap(V.col(V.cols() - 1));
            VA.col(0).swap(VA.col(VA.cols() - 1));

            V.conservativeResize(Eigen::NoChange, 1);
            VA.conservativeResize(Eigen::NoChange, 1);


            // The subspace matrix should now again be a matrix of dimension (1,1)
            const double last_diagonal_element = M(M.rows() - 1,M.cols() - 1);
            M = Eigen::MatrixXd::Constant(1, 1, last_diagonal_element);
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
        if (r.norm() < this->residue_tolerance) {
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


}  // namespace numopt
