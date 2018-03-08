#ifndef NUMOPT_COMMON_HPP
#define NUMOPT_COMMON_HPP

#include <Eigen/Dense>


/*
 *  TYPEDEFS
 */

namespace numopt {


typedef std::function<Eigen::VectorXd (const Eigen::VectorXd&)> VectorFunction;
typedef std::function<Eigen::MatrixXd (const Eigen::VectorXd&)> Jacobian;


}  // namespace numopt



#endif  // NUMOPT_COMMON_HPP
