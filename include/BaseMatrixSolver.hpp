#ifndef NUMOPT_BASEMATRIXSOLVER_HPP
#define NUMOPT_BASEMATRIXSOLVER_HPP



#include "BaseEigenproblemSolver.hpp"



namespace numopt {
namespace eigenproblem {


class BaseMatrixSolver : public numopt::eigenproblem::BaseEigenproblemSolver {
public:
    // CONSTRUCTOR
    /**
     *   Constructor based on the dimension @param dim of the eigenvalue problem.
     */
    explicit BaseMatrixSolver(size_t dim);

    // DESTRUCTOR
    ~BaseMatrixSolver() override = default;


    // PUBLIC PURE VIRTUAL METHODS
    /**
     *  Add @param value to the matrix at (@param index1, @param index2).
     */
    virtual void addToMatrix(double value, size_t index1, size_t index2) = 0;
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_BASEMATRIXSOLVER_HPP
