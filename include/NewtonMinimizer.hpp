#ifndef NUMOPT_NEWTONMINIMIZER_HPP
#define NUMOPT_NEWTONMINIMIZER_HPP



#include "BaseMinimizer.hpp"
#include "common.hpp"



namespace numopt {
namespace minimization {


class NewtonMinimizer : public BaseMinimizer {
private:
    const GradientFunction grad;
    const HessianFunction H;


public:
    // CONSTRUCTORS
    /**
     *  Constructor based on a given initial guess @param x0, a callable gradient function @param grad, a callable
     *  Hessian function @param H, and a @param convergence_threshold
     */
    NewtonMinimizer(const Eigen::VectorXd& x0, const GradientFunction& grad, const HessianFunction& H,
                    double convergence_threshold = 1.0e-08);


    // DESTRUCTOR
    ~NewtonMinimizer() override = default;


    // PUBLIC OVERRIDDEN FUNCTIONS
    /**
     *  Minimize the function f(x)
     *
     *  If successful, sets
     *      - @member is_solved to true
     *      - @member x to the found solution
     */
    void solve() override;
};


}  // namespace minimization
}  // numopt



#endif  // NUMOPT_NEWTONMINIMIZER_HPP
