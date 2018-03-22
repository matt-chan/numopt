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
    double ref_lowest_eigenvalue = 0.979;

    Eigen::VectorXd ref_lowest_eigenvector (5);
    ref_lowest_eigenvector << 0.994, -0.083, -0.042, -0.042, -0.042;


    // Solve using the Davidson diagonalization, supplying an initial guess
    Eigen::VectorXd t_0 (5);
    t_0 << 1, 0, 0, 0, 0;
    std::cout << "Did everything before DavidsonSolver instance" << std::endl;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, t_0);
    std::cout << "Made a DavidsonSolver instance" << std::endl;
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 0.005);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 0.005));
}


BOOST_AUTO_TEST_CASE( liu_50 ){

    // Let's prepare the Liu reference test (liu1978)
    size_t N = 50;
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(N, N);
    for (size_t i = 0; i < N; i++) {
        if (i < 5) {
            A(i, i) = 1 + 0.1 * i;
        } else {
            A(i, i) = 2 * (i + 1) - 1;
        }
    }


    // Solve the eigenvalue problem with Eigen
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver (A);
    double ref_lowest_eigenvalue = eigensolver.eigenvalues()(0);
    Eigen::VectorXd ref_lowest_eigenvector = eigensolver.eigenvectors().col(0);


    // Solve using the Davidson diagonalization, supplying an initial guess
    Eigen::VectorXd t_0 = Eigen::VectorXd::Zero(N);
    t_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, t_0);
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 1.0e-08));
}


BOOST_AUTO_TEST_CASE( liu_1000 ){

    // Let's prepare the Liu reference test (liu1978)
    size_t N = 1000;
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(N, N);
    for (size_t i = 0; i < N; i++) {
        if (i < 5) {
            A(i, i) = 1 + 0.1 * i;
        } else {
            A(i, i) = 2 * (i + 1) - 1;
        }
    }


    // Solve the eigenvalue problem with Eigen
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver (A);
    double ref_eigenvalue = eigensolver.eigenvalues()(0);
    Eigen::VectorXd ref_eigenvector = eigensolver.eigenvectors().col(0);


    // Solve using the Davidson diagonalization, supplying an initial guess
    Eigen::VectorXd t_0 = Eigen::VectorXd::Zero(N);
    t_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, t_0);
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_eigenvector, 1.0e-08));
}
