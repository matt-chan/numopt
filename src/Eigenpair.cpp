#include "Eigenpair.hpp"


namespace numopt {
namespace eigenproblem {


/*
 *  CONSTRUCTORS
 */
/**
 *  Default constructor based on a given @param dimension
 */
Eigenpair::Eigenpair(size_t dimension) :
    eigenvalue (0.0),
    eigenvector (Eigen::VectorXd::Zero(dimension))
{}


/**
 *  Constructor based on a given @param eigenvalue and @param eigenvector
 */
Eigenpair::Eigenpair(double eigenvalue, const Eigen::VectorXd& eigenvector) :
    eigenvalue (eigenvalue),
    eigenvector (eigenvector)
{}


}  // namespace eigenproblem
}  // namespace numopt
