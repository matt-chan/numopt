#ifndef NUMOPT_SOLVERTYPE_HPP
#define NUMOPT_SOLVERTYPE_HPP



namespace numopt {
namespace eigenproblem {


/**
 *  An enum class for the implemented solver types.
 */
enum class SolverType {
    DENSE,
    SPARSE,
    DAVIDSON
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_SOLVERTYPE_HPP
