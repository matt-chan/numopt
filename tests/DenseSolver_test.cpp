#define BOOST_TEST_MODULE "Dense"

#include "DenseSolver.hpp"

#include "cpputil.hpp"


#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



BOOST_AUTO_TEST_CASE ( diagonal_getter_dense ) {

    // Test the diagonal getter for a dense matrix
    Eigen::VectorXd ref_diagonal (100);

    numopt::eigenproblem::DenseSolver dense_solver (100);

    for (size_t i = 0; i < 100; i++) {
        dense_solver.addToMatrix(2*i, i, i);
        ref_diagonal(i) = 2*i;
    }


    BOOST_CHECK(ref_diagonal.isApprox(dense_solver.get_diagonal(), 1.0e-12));
}


BOOST_AUTO_TEST_CASE ( simple_dense ) {

    // Construct a random symmetric matrix and diagonalize it
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(10,10);
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd B = A + AT;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> self_adjoint_eigensolver (B);
    double ref_lowest_eigenvalue = self_adjoint_eigensolver.eigenvalues()(0);
    Eigen::VectorXd ref_lowest_eigenvector = self_adjoint_eigensolver.eigenvectors().col(0);


    // Add the matrix into the dense solver and find the lowest eigenpair using the dense solver
    numopt::eigenproblem::DenseSolver dense_solver (10);
    for (size_t i = 0; i < 10; i++) {
        for (size_t j = 0; j < 10; j++) {
            dense_solver.addToMatrix(B(i,j), i, j);
        }
    }

    dense_solver.solve();
    double test_lowest_eigenvalue = dense_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = dense_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(dense_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}
