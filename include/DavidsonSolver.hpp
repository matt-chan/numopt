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
    const Eigen::VectorXd t_0;  // the initial guess



public:
    // CONSTRUCTORS
    /**
     *  Constructor based on a given matrix-vector product function @param matrixVectorProduct, a @param diagonal,
     *  and initial guess @param t_0.
     */
    DavidsonSolver(const numopt::VectorFunction& matrixVectorProduct, const Eigen::VectorXd& diagonal,
                   const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08,
                   double correction_threshold = 1.0e-12, size_t maximum_subspace_dimension = 15,
                   size_t collapsed_subspace_dimension = 2);

    /**
     *  Constructor based on a given matrix @param A and an initial guess @param t_0
     */
    DavidsonSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& t_0, double residue_tolerance = 1.0e-08,
                   double correction_threshold = 1.0e-12, size_t maximum_subspace_dimension = 15,
                   size_t collapsed_subspace_dimension = 2);


    // DESTRUCTOR
    ~DavidsonSolver() override = default;


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
