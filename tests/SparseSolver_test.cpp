#define BOOST_TEST_MODULE "Sparse"

#include "SparseSolver.hpp"

#include <SymEigsSolver.h>
#include <MatOp/SparseSymMatProd.h>

#include "cpputil.hpp"


#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>  // include this to get main(), otherwise the compiler will complain



BOOST_AUTO_TEST_CASE ( diagonal_getter_sparse ) {

    // Test the diagonal getter for a sparse matrix
    Eigen::VectorXd ref_diagonal (100);

    numopt::eigenproblem::SparseSolver sparse_solver (100);

    for (size_t i = 0; i < 100; i++) {
        sparse_solver.addToMatrix(2*i, i, i);
        ref_diagonal(i) = 2*i;
    }


    BOOST_CHECK(ref_diagonal.isApprox(sparse_solver.get_diagonal(), 1.0e-12));
}


BOOST_AUTO_TEST_CASE ( simple_sparse ) {

    // Create a random sparse symmetric matrix (adapted from https://stackoverflow.com/a/30742847/7930415).
    // Also, put it in the sparse matrix solver.
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist (0.0,1.0);

    size_t rows = 100;
    size_t cols = 100;


    std::vector<Eigen::Triplet<double>> triplet_list;  // needed for Eigen::SparseMatrix<double>
    numopt::eigenproblem::SparseSolver sparse_solver (100);


    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < i; j++) {
            auto random_number = dist(gen);

            if (random_number > 0.5) {  // if larger than a threshold, insert it (symmetrically)
                triplet_list.emplace_back(static_cast<int>(i), static_cast<int>(j), random_number);
                triplet_list.emplace_back(static_cast<int>(j), static_cast<int>(i), random_number);

                sparse_solver.addToMatrix(random_number, i, j);
                sparse_solver.addToMatrix(random_number, j, i);
            }
        }
    }

    Eigen::SparseMatrix<double> A (rows, cols);
    A.setFromTriplets(triplet_list.begin(), triplet_list.end());


    // Find the lowest eigenpair using Spectra
    Spectra::SparseSymMatProd<double> matrixVectorProduct (A);
    Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE, Spectra::SparseSymMatProd<double>> spectra_sparse_eigensolver (&matrixVectorProduct, 1, 3);  // request 1 eigenpair, and use 3 Ritz pairs for the solution (need at least 2 more Ritz pairs than requested eigenvalues)
    spectra_sparse_eigensolver.init();
    spectra_sparse_eigensolver.compute();

    double ref_lowest_eigenvalue = spectra_sparse_eigensolver.eigenvalues()(0);
    Eigen::VectorXd ref_lowest_eigenvector = spectra_sparse_eigensolver.eigenvectors().col(0);


    // Find the lowest eigenpair using the sparse solver
    sparse_solver.solve();
    double test_lowest_eigenvalue = sparse_solver.get_eigenvalue();
    Eigen::VectorXd test_lowest_eigenvector = sparse_solver.get_eigenvector();


    BOOST_CHECK(std::abs(test_lowest_eigenvalue - ref_lowest_eigenvalue) < 1.0e-08);
    BOOST_CHECK(cpputil::linalg::areEqualEigenvectors(test_lowest_eigenvector, ref_lowest_eigenvector, 1.0e-08));


    // Check if the eigenvector is normalized
    BOOST_CHECK(std::abs(sparse_solver.get_eigenvector().norm() - 1) < 1.0e-12);
}


BOOST_AUTO_TEST_CASE ( simple_sparse_number_of_requested_eigenpairs ) {

    size_t number_of_requested_eigenpairs = 5;

    // Create a random sparse symmetric matrix (adapted from https://stackoverflow.com/a/30742847/7930415).
    // Also, put it in the sparse matrix solver.
    std::default_random_engine gen;
    std::uniform_real_distribution<double> dist(0.0,1.0);

    size_t rows = 100;
    size_t cols = 100;


    std::vector<Eigen::Triplet<double>> triplet_list;  // needed for Eigen::SparseMatrix<double>
    numopt::eigenproblem::SparseSolver sparse_solver (rows, number_of_requested_eigenpairs);


    for (size_t i = 0; i < rows; i++) {
        for (size_t j = 0; j < i; j++) {
            auto random_number = dist(gen);

            if (random_number > 0.5) {  // if larger than a threshold, insert it (symmetrically)
                triplet_list.emplace_back(static_cast<int>(i), static_cast<int>(j), random_number);
                triplet_list.emplace_back(static_cast<int>(j), static_cast<int>(i), random_number);

                sparse_solver.addToMatrix(random_number, i, j);
                sparse_solver.addToMatrix(random_number, j, i);
            }
        }
    }

    Eigen::SparseMatrix<double> A (rows, cols);
    A.setFromTriplets(triplet_list.begin(), triplet_list.end());


    // Find the lowest eigenpairs using Spectra
    Spectra::SparseSymMatProd<double> matrixVectorProduct (A);
    Spectra::SymEigsSolver<double, Spectra::SMALLEST_ALGE, Spectra::SparseSymMatProd<double>> spectra_sparse_eigensolver (&matrixVectorProduct, number_of_requested_eigenpairs, number_of_requested_eigenpairs+3);  // number_of_requested_eigenpairs + 3 Ritz pairs for the solution (need at least 2 more Ritz pairs than requested eigenvalues)
    spectra_sparse_eigensolver.init();
    spectra_sparse_eigensolver.compute();

    Eigen::VectorXd ref_lowest_eigenvalues = spectra_sparse_eigensolver.eigenvalues().head(number_of_requested_eigenpairs);
    Eigen::MatrixXd ref_lowest_eigenvectors = spectra_sparse_eigensolver.eigenvectors().topLeftCorner(rows, number_of_requested_eigenpairs);

//    // Create eigenpairs for the reference eigenpairs
//    std::vector<numopt::eigenproblem::Eigenpair> ref_eigenpairs (number_of_requested_eigenpairs);
//    for (size_t i = 0; i < number_of_requested_eigenpairs; i++) {
//        ref_eigenpairs[i] = numopt::eigenproblem::Eigenpair(ref_lowest_eigenvalues(i), ref_lowest_eigenvectors.col(i));
//    }
//
//
//    // Find the lowest eigenpairs using our sparse solver
//    sparse_solver.solve();
//    std::vector<numopt::eigenproblem::Eigenpair> eigenpairs = sparse_solver.get_eigenpairs();



//    for (size_t i = 0; i < number_of_requested_eigenpairs; i++) {
//        BOOST_CHECK(eigenpairs[i].isEqual(ref_eigenpairs[i]));  // check if the found eigenpairs are equal to the reference eigenpairs
//        // BOOST_CHECK(std::abs(eigenpairs[i].get_eigenvector().norm() - 1) < 1.0e-11);  // check if the found eigenpairs are normalized
//    }
}
