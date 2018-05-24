#include "NewtonMinimizer.hpp"

#include "common.hpp"
#include "NewtonSystemOfEquationsSolver.hpp"



#include <iostream>


namespace numopt {
namespace minimization {


/*
 *  CONSTRUCTORS
 */
/**
 *  Constructor based on a given initial guess @param x0, a callable gradient function @param grad, a callable
 *  Hessian function @param H, and a @param convergence_threshold
 */
NewtonMinimizer::NewtonMinimizer(const Eigen::VectorXd& x0, const GradientFunction& grad, const HessianFunction& H,
                                 double convergence_threshold) :
    BaseMinimizer(x0, convergence_threshold),
    grad (grad),
    H (H)
{}


/*
 *  PUBLIC OVERRIDDEN FUNCTIONS
 */
/**
 *  Minimize the function f(x)
 *
 *  If successful, sets
 *      - @member is_solved to true
 *      - @member x to the found solution
 */
void NewtonMinimizer::solve() {

    // Requiring the gradient to vanish, and then calculating the corresponding Newton step, is equivalent to solving
    // the system of equations grad(f(x)) = 0 using a Newton step

    // For mathematical correctness, the Jacobian of the gradient is the transpose of the Hessian of the scalar function
    // behind it
    numopt::JacobianFunction H_t = [this](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H = this->H(x);
        H.transposeInPlace();
        return H;
    };
    // We have defined an elaborate lambda function, because
    // numopt::JacobianFunction H_t = [this](const Eigen::VectorXd& x) { return this->H(x).transpose(); }
    // produces an error


    // For previously established reasons, we can use the NewtonSystemOfEquationsSolver as an implementation of this
    // minimization problem
    numopt::syseq::NewtonSystemOfEquationsSolver newton_syseq_solver (this->x0, this->grad, H_t, this->convergence_threshold);
    newton_syseq_solver.solve();


    // If we haven't found a solution, the error is raised inside the NewtonSystemOfEquationsSolver, so we are free to
    // assert that if the data flow gets to here, a solution is found
    this->is_solved = true;
    this->x = newton_syseq_solver.get_solution();
}


}  // namespace minimization
}  // namespace numopt
