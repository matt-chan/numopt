#ifndef NUMOPT_DENSESOLVER_HPP
#define NUMOPT_DENSESOLVER_HPP



#include "BaseMatrixSolver.hpp"



namespace numopt {
namespace eigenproblem {


class DenseSolver : public numopt::eigenproblem::BaseMatrixSolver {
private:
    Eigen::MatrixXd matrix;


public:
    // CONSTRUCTOR
    /**
     *   Constructor based on the dimension @param dim of the eigenvalue problem.
     */
    explicit DenseSolver(size_t dim);


    // DESTRUCTOR
    ~DenseSolver() override = default;


    // PUBLIC OVERRIDDEN METHODS
    /**
     *  Solve the full dense eigenvalue problem of @member matrix.
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the lowest calculated eigenvalue
     *      - @member eigenvector to the associated eigenvector
     */
    void solve() override;

    /**
     *  Add @param value to the matrix at (@param index1, @param index2).
     */
    void addToMatrix(double value, size_t index1, size_t index2) override;


    // GETTERS
    Eigen::MatrixXd get_matrix(){return this->matrix;};

};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_DENSESOLVER_HPP
