#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <cassert>
#include <cstddef>
#include <utility>

namespace mat {

constexpr double PI = 3.14159265358979323846;

/*
 * A light-weight matrix implementation
 * Data is stored in a flat vector in row-major order
 */
template <typename T>
class Matrix {
public:
    // Constructors
    // Default constructor
    Matrix() = default;

    // Zero initialize the matrix data
    Matrix(std::size_t rows, std::size_t cols)
        : rows_(rows), cols_(cols), data_(rows * cols) {}

    // Set all values to `val`
    Matrix(std::size_t rows, std::size_t cols, const T& value)
        : rows_(rows), cols_(cols), data_(rows * cols, value) {}

    // Copy constructors
    Matrix(const Matrix&) = default;
    Matrix& operator=(const Matrix&) = default;     // Copy assignment

    // Move constructors
    Matrix(Matrix&& other) noexcept
        :   data_(std::move(other.data_)),
            rows_(other.rows_),
            cols_(other.cols_)
        {
            other.rows_ = 0;
            other.cols_ = 0;
        }

    Matrix& operator=(Matrix&& other) noexcept {    // Move assignment
        if (this != &other) {
            data_ = std::move(other.data_); 
            rows_ = other.rows_; other.rows_ = 0;
            cols_ = other.cols_; other.cols_ = 0;
        }

        return *this;
    }

    // Constructor for using data from an existing vector
    Matrix(std::size_t rows, std::size_t cols, std::vector<T> vec)
        : rows_(rows), cols_(cols), data_(std::move(vec))
    {
        assert(rows_ * cols_ == data_.size());
    }

    /* Replace the old data with vec */
    void assign(std::size_t rows, std::size_t cols, std::vector<T> vec) {
        assert(rows * cols == vec.size());
        rows_ = rows;
        cols_ = cols;
        data_ = std::move(vec);
    }

    // Getters for accessing the attributes
    inline std::size_t rows() const { return rows_; }
    inline std::size_t cols() const { return cols_; }

    // Getters for accessing the data
    inline T& operator() (std::size_t i, std::size_t j) { return data_[i*cols_ + j]; }
    inline const T& operator() (std::size_t i, std::size_t j) const { return data_[i*cols_ + j]; }

    // Getters to give reference to the complete underlying data
    inline std::vector<T>& data() { return data_; }
    inline const std::vector<T>& data() const { return data_; }

    // Getters for raw data
    inline T* rawData() { return data_.data(); }
    inline const T* rawData() const { return data_.data(); }

    // Some helper functions
    void addRow(std::vector<T>& row) {
        assert(row.size() == this->cols_);

        for (auto& t: row) this->data_.push_back(t);
    }

    void appendRows(mat::Matrix<T>&& other) {
        if (this->rows_ == 0 && this->cols_ == 0) {
            this->rows_ = other.rows_; other.rows_ = 0;
            this->cols_ = other.cols_; other.cols_ = 0;
            this->data_ = std::move(other.data_);
            return;
        }

        assert(this->cols_ == other.cols_);

        this->data_.insert(
            this->data_.end(),
            std::make_move_iterator(other.data_.begin()),
            std::make_move_iterator(other.data_.end())
        );

        this->rows_ += other.rows_; other.rows_ = 0;
        other.cols_ = 0;
        return;
    }

private:
    std::size_t     rows_ = 0;
    std::size_t     cols_ = 0;
    std::vector<T>  data_;
};

/* Returns the result of A+B */
template <typename T>
Matrix<T> operator+ (const Matrix<T>& A, const Matrix<T>& B) {
    assert(A.rows() == B.rows());
    assert(A.cols() == B.cols());

    Matrix<T> C(A.rows(), A.cols());

    const std::vector<T>& a = A.data();
    const std::vector<T>& b = B.data();
    std::vector<T>& c = C.data();

    for (std::size_t i = 0; i < c.size(); i++)
        c[i] = a[i] + b[i];

    return C;
}

/* Retruns the result of A-B */
template <typename T>
Matrix<T> operator- (const Matrix<T>& A, const Matrix<T>& B) {
    assert(A.rows() == B.rows());
    assert(A.cols() == B.cols());

    Matrix<T> C(A.rows(), A.cols());

    const std::vector<T>& a = A.data();
    const std::vector<T>& b = B.data();
    std::vector<T>& c = C.data();

    for (std::size_t i = 0; i < c.size(); i++)
        c[i] = a[i] - b[i];

    return C;
}

/*
 * Returns the result of scaling every element by `k`
 * Result B: B[i,j] = k * A[i,j]
 */
template <typename T>
Matrix<T> operator* (const Matrix<T>& A, const T& k) {
    Matrix<T> C(A.rows(), A.cols());

    const std::vector<T>& a = A.data();
    std::vector<T>& c = C.data();

    for (std::size_t i = 0; i < c.size(); i++)
        c[i] = a[i] * k;

    return C;
}

/* Returns the transpose of A */
template <typename T>
Matrix<T> transpose (const Matrix<T>& A) {
    Matrix<T> C(A.cols(), A.rows());

    for (std::size_t i = 0; i < A.rows(); i++) {
        for (std::size_t j = 0; j < A.cols(); j++)
            C(j, i) = A(i, j);
    }

    return C;
}

/* Calculates the reuslt of matrix multiplication of A and B */
template <typename T>
Matrix<T> operator* (const Matrix<T>& A, const Matrix<T>& B) {
    assert(A.cols() == B.rows());

    Matrix<T> C(A.rows(), B.cols(), T{});

    const T* a = A.rawData();
    const T* b = B.rawData();
    T* c = C.rawData();

    for (std::size_t i = 0; i < A.rows(); i++) {
        for (std::size_t k = 0; k < A.cols(); k++) {
            T a_ik = a[i*A.cols() + k];
            for (std::size_t j = 0; j < B.cols(); j++) {
                c[i*C.cols() + j] += a_ik * b[k*B.cols() + j];
            }
        }
    }

    return C;
}

/*
 * Computes the result of matrix multiplication of A and B
 * Stores the result in C
 * Ensure that C is cleared before passing to the funciton else the result will be C += A*B
 */
template <typename T>
void matmul (const Matrix<T>& A, const Matrix<T>& B, Matrix<T>& C) {
    assert(A.cols() == B.rows());
    assert(A.rows() == C.rows());
    assert(B.cols() == C.cols());

    const T* a = A.rawData();
    const T* b = B.rawData();
    T* c = C.rawData();

    for (std::size_t i = 0; i < A.rows(); i++) {
        for (std::size_t k = 0; k < A.cols(); k++) {
            T a_ik = a[i*A.cols() + k];
            for (std::size_t j = 0; j < B.cols(); j++) {
                c[i*C.cols() + j] += a_ik * b[k*B.cols() + j];
            }
        }
    }
}

/*
 * Calculates the reuslt of element-wise matrix multiplication of A and B
 * Result C: C[i,j] = A[i,j] * B[i,j]
 */
template <typename T>
Matrix<T> elementwiseProduct (const Matrix<T>& A, const Matrix<T>& B) {
    assert(A.rows() == B.rows());
    assert(A.cols() == B.cols());

    Matrix<T> C(A.rows(), A.cols(), T{});

    const T* a = A.rawData();
    const T* b = B.rawData();
    T* c = C.rawData();

    for (std::size_t i = 0; i < A.rows(); i++) {
        for (std::size_t j = 0; j < A.cols(); j++) {
            c[i*C.cols() + j] = a[i*A.cols() + j] * b[i*B.cols() + j];
        }
    }

    return C;
}

/*
 * Calculates the reuslt of element-wise matrix division of A and B
 * Result C: C[i,j] = A[i,j] / B[i,j]
 */
template <typename T>
Matrix<T> elementwiseDivision (const Matrix<T>& A, const Matrix<T>& B) {
    assert(A.rows() == B.rows());
    assert(A.cols() == B.cols());

    Matrix<T> C(A.rows(), A.cols(), T{});

    const T* a = A.rawData();
    const T* b = B.rawData();
    T* c = C.rawData();

    for (std::size_t i = 0; i < A.rows(); i++) {
        for (std::size_t j = 0; j < A.cols(); j++) {
                c[i*C.cols() + j] = a[i*A.cols() + j] / b[i*B.cols() + j];
        }
    }

    return C;
}

template <typename T>
Matrix<T> linspace (const T& start, const T& end, const std::size_t& num) {
    assert(num >= 2);

    auto step = (static_cast<double>(end) - static_cast<double>(start)) / static_cast<double>(num - 1);

    Matrix<T> res(1, num);
    res(0, 0) = start;
    res(0, num-1) = end;

    for (std::size_t i = 1; i < num-1; i++) res(0, i) = start + i * step;

    return res;
}


}

#endif // MATRIX_H
