#define BOOST_TEST_MODULE "DavidsonSolver"

#include "DavidsonSolver.hpp"

#include "cpputil.hpp"


#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



BOOST_AUTO_TEST_CASE ( esqc_example_solver ) {

    // We can find the following example at (http://www.esqc.org/static/lectures/Malmqvist_2B.pdf)


    // Build up the example matrix
    Eigen::MatrixXd A = Eigen::MatrixXd::Constant(5, 5, 0.1);
    A(0,0) = 1.0;
    A(1,1) = 2.0;
    A(2,2) = 3.0;
    A(3,3) = 3.0;
    A(4,4) = 3.0;


    // The solutions to the problem are given in the example
    double ref_eigenvalue = 0.979;

    Eigen::VectorXd ref_eigenvector (5);
    ref_eigenvector << 0.994, -0.083, -0.042, -0.042, -0.042;


    // Solve using the Davidson diagonalization, supplying an initial guess
    Eigen::VectorXd t_0 (5);
    t_0 << 1, 0, 0, 0, 0;
    numopt::DavidsonSolver davidson_solver (A, t_0);
    davidson_solver.solve();

    double test_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_eigenvector = davidson_solver.get_eigenvector();


    std::cout << test_eigenvalue << std::endl;
    std::cout << test_eigenvector << std::endl;
    BOOST_CHECK(std::abs(test_eigenvalue - ref_eigenvalue) < 0.005);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_eigenvector, ref_eigenvector, 0.005));
}