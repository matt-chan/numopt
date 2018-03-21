#ifndef NUMOPT_BASEEIGENVALUESOLVER_HPP
#define NUMOPT_BASEEIGENVALUESOLVER_HPP



#include <cstddef>
#include <Eigen/Dense>



namespace numopt {
namespace eigenproblem {


class BaseEigenproblemSolver {
protected:
    const size_t dim;  // the dimension of the vector space associated to the eigenvalue problem

    bool is_solved = false;
    double eigenvalue;
    Eigen::VectorXd eigenvector;


    // PROTECTED CONSTRUCTORS
    /**
     *  Protected constructor to initialize the const @member dim by @param dim.
     */
    explicit BaseEigenproblemSolver(size_t dim);


public:
    // DESTRUCTOR
    virtual ~BaseEigenproblemSolver() = default;


    // PUBLIC PURE VIRTUAL METHODS
    /**
     *  Solve the eigenvalue problem associated to the eigenproblem solver.
     *
     *  If successful, it sets
     *      - @member is_solved to true
     *      - @member eigenvalue to the lowest calculated eigenvalue
     *      - @member eigenvector to the associated eigenvector
     */
    virtual void solve() = 0;


    // GETTERS
    double get_eigenvalue() const;
    Eigen::VectorXd get_eigenvector() const;
};


}  // namespace eigenproblem
}  // namespace numopt





#endif  // NUMOPT_BASEEIGENVALUESOLVER_HPP
