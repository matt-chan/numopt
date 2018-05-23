#define BOOST_TEST_MODULE "NewtonDescentVector"

#include "VectorNewtonDescent.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



/*
 *  TEST FUNCTION DEFINITIONS
 */

/**
 *  Implement a simple vector function that returns (x.x, 2x.x)
 */
Eigen::VectorXd f(const Eigen::VectorXd& x) {

    Eigen::VectorXd f (2);

    f << x(0) * x(0) + x(1) * x(1),
         2 * x(0) * x(0) + 2 * x(1) * x(1);

    return f;
}


/**
 *  Implement the Jacobian of the previous function
 */
Eigen::MatrixXd J(const Eigen::VectorXd& x) {

    Eigen::MatrixXd J (2, 2);

    J << 2 * x(0), 2 * x(1),
         4 * x(0), 4 * x(1);

    return J;
}



/*
 *  BOOST UNIT TESTS
 */

BOOST_AUTO_TEST_CASE ( norm_squared_function ) {

    // Test that the previous implementations actually work by checking the values at x=(1,1)
    Eigen::VectorXd f_test (2);
    f_test << 2, 4;

    Eigen::MatrixXd J_test (2, 2);
    J_test << 2, 2,
              4, 4;

    Eigen::VectorXd x_test (2);
    x_test << 1, 1;

    BOOST_REQUIRE(f_test.isApprox(f(x_test), 1.0e-8));
    BOOST_REQUIRE(J_test.isApprox(J(x_test), 1.0e-8));


    // Do the numerical optimization
    Eigen::VectorXd x0 (2);
    x0 << 3, 2;
    numopt::VectorNewtonDescent newton_vector_opt (x0, f, J);  // apparently, the compiler can convert to numopt::VectorFunction and numopt::JacobianFunction
    newton_vector_opt.solve();
    Eigen::VectorXd solution = newton_vector_opt.get_solution();


    BOOST_CHECK(solution.isZero(1.0e-08));  // the analytical minimum of this function is at (0,0)
}
