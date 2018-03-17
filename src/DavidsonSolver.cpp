#include "DavidsonSolver.hpp"



namespace numopt {


double DavidsonSolver::get_eigenvalue() const { return this->eigenvalue; }
Eigen::VectorXd DavidsonSolver::get_eigenvector() const { return this->eigenvector; }



}  // namespace numopt



