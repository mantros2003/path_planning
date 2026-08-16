#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "matrix.h"

using namespace std;
using namespace std::chrono;

constexpr double EPS = 1e-9;

using Matrix = mat::Matrix<double>;

// Random matrix
Matrix randomMatrix(size_t rows, size_t cols)
{
    static mt19937 rng(42);
    uniform_real_distribution<double> dist(-10.0, 10.0);

    Matrix A(rows, cols);

    for (double &x : A.data())
        x = dist(rng);

    return A;
}

// Comparison
bool equal(const Matrix& A, const Matrix& B)
{
    if (A.rows() != B.rows())
        return false;

    if (A.cols() != B.cols())
        return false;

    for (size_t i = 0; i < A.data().size(); i++)
    {
        if (fabs(A.data()[i] - B.data()[i]) > EPS)
            return false;
    }

    return true;
}

// Reference multiplication (i-j-k)
Matrix multiply_reference(const Matrix& A, const Matrix& B)
{
    Matrix C(A.rows(), B.cols(), 0.0);

    for (size_t i = 0; i < A.rows(); i++)
    {
        for (size_t j = 0; j < B.cols(); j++)
        {
            double sum = 0;

            for (size_t k = 0; k < A.cols(); k++)
                sum += A(i, k) * B(k, j);

            C(i, j) = sum;
        }
    }

    return C;
}

// i-k-j
Matrix multiply_ikj(const Matrix& A, const Matrix& B)
{
    Matrix C(A.rows(), B.cols(), 0.0);

    for (size_t i = 0; i < A.rows(); i++)
    {
        for (size_t k = 0; k < A.cols(); k++)
        {
            double aik = A(i, k);

            for (size_t j = 0; j < B.cols(); j++)
                C(i, j) += aik * B(k, j);
        }
    }

    return C;
}

// k-i-j
Matrix multiply_kij(const Matrix& A, const Matrix& B)
{
    Matrix C(A.rows(), B.cols(), 0.0);

    for (size_t k = 0; k < A.cols(); k++)
    {
        for (size_t i = 0; i < A.rows(); i++)
        {
            double aik = A(i, k);

            for (size_t j = 0; j < B.cols(); j++)
                C(i, j) += aik * B(k, j);
        }
    }

    return C;
}

// Timing
template<typename Func>
double benchmark(Func f, int iterations = 10)
{
    f();

    double total = 0;

    for (int i = 0; i < iterations; i++)
    {
        auto start = high_resolution_clock::now();

        f();

        auto end = high_resolution_clock::now();

        total += duration<double, milli>(end - start).count();
    }

    return total / iterations;
}

// Main
int main()
{
    constexpr size_t N = 512;

    Matrix A = randomMatrix(N, N);
    Matrix B = randomMatrix(N, N);

    cout << "Computing reference...\n";

    Matrix reference = multiply_reference(A, B);

    cout << "Checking correctness...\n";

    {
        Matrix C = multiply_ikj(A, B);

        cout << "i-k-j : "
             << (equal(reference, C) ? "PASS" : "FAIL")
             << '\n';
    }

    {
        Matrix C = multiply_kij(A, B);

        cout << "k-i-j : "
             << (equal(reference, C) ? "PASS" : "FAIL")
             << '\n';
    }

    {
        Matrix C(A.rows(), B.cols(), 0.0);

        mat::matmul(A, B, C);

        cout << "Library : "
             << (equal(reference, C) ? "PASS" : "FAIL")
             << '\n';
    }

    cout << "\nBenchmark\n";
    cout << "-----------------------------\n";

    auto t_ref = benchmark([&]()
    {
        volatile Matrix C = multiply_reference(A, B);
    });

    auto t_ikj = benchmark([&]()
    {
        volatile Matrix C = multiply_ikj(A, B);
    });

    auto t_kij = benchmark([&]()
    {
        volatile Matrix C = multiply_kij(A, B);
    });

    auto t_lib = benchmark([&]()
    {
        Matrix C(A.rows(), B.cols(), 0.0);
        mat::matmul(A, B, C);
    });

    cout << fixed << setprecision(3);

    cout << "Reference (i-j-k): " << t_ref << " ms\n";
    cout << "i-k-j            : " << t_ikj << " ms\n";
    cout << "k-i-j            : " << t_kij << " ms\n";
    cout << "Library          : " << t_lib << " ms\n";
}
