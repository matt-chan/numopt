#include "Eigenpair.hpp"

#include <cpputil.hpp>



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



/*
 *  PUBLIC METHODS
 */

/**
 *  @return if, within a given @param tolerance, this is equal to @param other
 *  This is the case if the eigenvalues are equal given the @param tolerance, and so are the eigenvectors
 */
bool Eigenpair::isEqual(const numopt::eigenproblem::Eigenpair& other, double tolerance) const {


    if (this->eigenvector.size() != other.get_eigenvector().size()) {
        throw std::invalid_argument("Can't compare eigenpairs with eigenvectors of different dimension.");
    }

    if (std::abs(this->eigenvalue - other.eigenvalue) < tolerance) {
        if (cpputil::linalg::areEqualEigenvectors(this->eigenvector, other.get_eigenvector(), tolerance)) {
            return true;
        }
    }

    return false;
}


}  // namespace eigenproblem
}  // namespace numopt
