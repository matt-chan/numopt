#ifndef NUMOPT_SPARSESOLVER_HPP
#define NUMOPT_SPARSESOLVER_HPP



#include <Eigen/Sparse>

#include "BaseMatrixSolver.hpp"



namespace numopt {
namespace eigenproblem {


class SparseSolver : public numopt::eigenproblem::BaseMatrixSolver {
private:
    Eigen::SparseMatrix<double> matrix;


public:
    // CONSTRUCTORS
    /**
     *   Constructor based on the dimension @param dim of the eigenvalue problem.
     */
    explicit SparseSolver(size_t dim);


    // DESTRUCTOR
    ~SparseSolver() override = default;


    // PUBLIC OVERRIDDEN METHODS
    /**
     *  Solve the sparse eigenvalue problem of @member matrix.
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
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_SPARSESOLVER_HPP
