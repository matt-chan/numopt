#define BOOST_TEST_MODULE "DavidsonSolver"

#include "DavidsonSolver.hpp"

#include "cpputil.hpp"


#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



BOOST_AUTO_TEST_CASE ( constructor ) {

    // Create an example matrix
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);

    // Test constructors with one supplied guess vector
    Eigen::VectorXd x_0 = Eigen::VectorXd::Constant(2, 1);
    BOOST_CHECK_THROW(numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0, 3, 1.0e-08, 1.0e-12, 4, 8), std::invalid_argument);  // 3 requested eigenpairs: not enough initial guesses
    BOOST_CHECK_THROW(numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0, 1, 1.0e-08, 1.0e-12, 4, 8), std::invalid_argument);  // collapsed subspace dimension (8) cannot be larger than maximum subspace dimension (4)
    BOOST_CHECK_NO_THROW(numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0));


    // Test a constructor with two supplied guess vectors
    Eigen::MatrixXd Y_0 = Eigen::MatrixXd::Identity(2, 2);
    BOOST_CHECK_THROW(numopt::eigenproblem::DavidsonSolver davidson_solver (A, Y_0, 2, 1.0e-08, 1.0e-12, 1, 8), std::invalid_argument);  // collapsed subspace dimension (1) cannot be smaller number of requested eigenpairs (2)
}


BOOST_AUTO_TEST_CASE ( diagonal_getter_Davidson ) {

    // Test the diagonal getter for Davidson
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd x_0 = Eigen::VectorXd::Constant(2, 1);

    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0);


    BOOST_CHECK(A.diagonal().isApprox(davidson_solver.get_diagonal(), 1.0e-12));
}


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
    Eigen::VectorXd x_0 (5);
    x_0 << 1, 0, 0, 0, 0;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0);
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 0.005);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 0.005));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(davidson_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}


BOOST_AUTO_TEST_CASE ( esqc_number_of_requested_eigenpairs ) {

    BOOST_CHECK(false);
}


// 12 iterations
BOOST_AUTO_TEST_CASE ( liu_50 ) {

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
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(N);
    x_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0);
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(davidson_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}


// Test a forced subspace collapse for small dimensions (maximum subspace dimension < 12)
BOOST_AUTO_TEST_CASE ( liu_50_collapse ) {

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
    double ref_eigenvalue = eigensolver.eigenvalues()(0);
    Eigen::VectorXd ref_eigenvector = eigensolver.eigenvectors().col(0);


    // Solve using the Davidson diagonalization, supplying an initial guess
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(N);
    x_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0, 1, 1.0e-08, 1.0e-12, 10, 5);  // maximum subspace dimension = 10, collapsed subspace dimension = 5
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(davidson_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}


BOOST_AUTO_TEST_CASE ( liu_50_number_of_requested_eigenpairs ) {

    BOOST_CHECK(false);
}


// 12 iterations for large dimensions
BOOST_AUTO_TEST_CASE ( liu_1000 ) {

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
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(N);
    x_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0);
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(davidson_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}


// Test a forced subspace collapse for large dimensions (maximum subspace dimension < 12)
BOOST_AUTO_TEST_CASE ( liu_1000_collapse ) {

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
    Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(N);
    x_0(0) = 1;
    numopt::eigenproblem::DavidsonSolver davidson_solver (A, x_0, 1, 1.0e-08, 1.0e-12, 10, 5);  // maximum subspace dimension = 10, collapsed subspace dimension = 5
    davidson_solver.solve();

    double test_lowest_eigenvalue = davidson_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = davidson_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(davidson_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}
