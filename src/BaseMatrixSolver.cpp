#include "BaseMatrixSolver.hpp"



namespace numopt {
namespace eigenproblem {



// CONSTRUCTOR
/**
 *   Constructor based on the dimension @param dim of the eigenvalue problem.
 */
BaseMatrixSolver::BaseMatrixSolver(size_t dim) :
    BaseEigenproblemSolver(dim)
{}



}  // namespace eigenproblem
}  // namespace numopt
