#ifndef NUMOPT_EIGENPAIR_HPP
#define NUMOPT_EIGENPAIR_HPP



#include <Eigen/Dense>



namespace numopt {
namespace eigenproblem {


/**
 *  A container class to store an eigenpair, i.e. an eigenvector with a corresponding eigenvalue
 */
class Eigenpair {
private:
    double eigenvalue;
    Eigen::VectorXd eigenvector;


public:
    // CONSTRUCTORS
    /**
     *  Initializing constructor based on a given @param dimension, this will:
     *      - set @member eigenvalue to 0.0
     *      - set @member eigenvector to an @param dimensional Eigen::VectorXd::Zero
     */
    Eigenpair(size_t dimension = 1);

    /**
     *  Constructor based on a given @param eigenvalue and @param eigenvector
     */
    Eigenpair(double eigenvalue, const Eigen::VectorXd& eigenvector);


    // GETTERS
    double get_eigenvalue() const { return this->eigenvalue; };
    Eigen::VectorXd get_eigenvector() const { return this->eigenvector; };
};


}  // namespace eigenproblem
}  // namespace numopt



#endif  // NUMOPT_EIGENPAIR_HPP
