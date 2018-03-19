#ifndef NUMOPT_DAVIDSONSOLVER_HPP
#define NUMOPT_DAVIDSONSOLVER_HPP



#include <Eigen/Dense>

#include "common.hpp"



namespace numopt {

/**
 *  A class that implements the Davidson algorithm for finding the lowest eigenpair of a (possibly large) diagonally-
 *  dominant symmetric matrix.
 */
class DavidsonSolver {
private:
    static constexpr size_t maximum_numer_of_iterations = 128;

    const double residue_tolerance;  // the tolerance on the norm of the residual vector
    const double correction_threshold;  // the threshold used in solving the (approximated) residue correction equation
    const size_t maximum_subspace_dimension;

    const Eigen::VectorXd diagonal;  // the diagonal of the matrix in question
    const Eigen::VectorXd t_0;  // the initial guess
    const numopt::VectorFunction matrixVectorProduct;
    const size_t dim;  // the dimension of the eigenvalue problem

    bool is_solved = false;
    double eigenvalue;
    Eigen::VectorXd eigenvector;


public:
    // CONSTRUCTORS
    /**
     *  Constructor based on a given matrix-vector product function @param matrixVectorProduct initial guess @param t_0.
     */
    DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& t_0, const Eigen::VectorXd& diagonal, double residue_tolerance = 1.0e-08, double correction_threshold = 1.0e-03, size_t maximum_subspace_dimension = 15);

    /**
     *  Constructor based on a given matrix @param A and an initial guess @param t_0
     */
    DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08, double correction_threshold = 1.0e-03, size_t maximum_subspace_dimension = 15);


    // GETTERS
    double get_eigenvalue() const;
    Eigen::VectorXd get_eigenvector() const;


    // PUBLIC METHODS
    /**
     *  Solve the eigenvalue problem related to the given matrix-vector product.
     *
     *  If successful, sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the calculated eigenvalue
     *      - @member eigenvector to the calculated eigenvector
     */
    double solve();
};


}  // namespace numopt







#endif  // NUMOPT_DAVIDSONSOLVER_HPP
