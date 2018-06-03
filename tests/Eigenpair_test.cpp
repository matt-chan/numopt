#define BOOST_TEST_MODULE "Eigenpair"

#include "Eigenpair.hpp"


#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



BOOST_AUTO_TEST_CASE ( isEqual ) {

    // Create some test eigenvectors
    Eigen::VectorXd u1 (2);
    u1 << 1, 0;
    Eigen::VectorXd u2 (2);
    u2 << -1, 0;
    Eigen::VectorXd v (2);
    v << 1, 1;
    Eigen::VectorXd w (3);
    w << 1, 0, 0;


    // Test isEqual for some eigenpairs
    numopt::eigenproblem::Eigenpair eigenpair1 (1, u1);
    numopt::eigenproblem::Eigenpair eigenpair2 (1, u2);
    numopt::eigenproblem::Eigenpair eigenpair3 (2, u1);
    numopt::eigenproblem::Eigenpair eigenpair4 (1, v);
    numopt::eigenproblem::Eigenpair eigenpair5 (1, w);

    BOOST_CHECK(eigenpair1.isEqual(eigenpair1));
    BOOST_CHECK(eigenpair1.isEqual(eigenpair2));  // eigenvectors are equal up to their sign

    BOOST_CHECK(!eigenpair1.isEqual(eigenpair3));  // 1 != 2
    BOOST_CHECK(!eigenpair1.isEqual(eigenpair4));  // u1 != v


    BOOST_CHECK_THROW(eigenpair1.isEqual(eigenpair5), std::invalid_argument);  // can't compare eigenvectors of different dimensions
}
