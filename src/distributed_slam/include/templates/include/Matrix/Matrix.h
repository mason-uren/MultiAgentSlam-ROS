//
// Created by Mason U'Ren on 2019-03-11.
//

#ifndef MULTIAGENTSLAM_MATRIX_H
#define MULTIAGENTSLAM_MATRIX_H

#define EIGEN_USE_MKL_ALL

#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>
#include <Eigen/SparseLU>
#include <Eigen/OrderingMethods>

template <class T = float>
class Matrix {
private:
    typedef Eigen::Triplet<T> triplet;

    Eigen::SparseMatrix<T> data{};
    uint16_t nRows{};
    uint16_t nCols{};
    bool trans{};

    class RowProxy {
        Matrix& _matrix;
        size_t _row;

    public:
        RowProxy(Matrix& matrix, size_t row) : _matrix(matrix), _row(row) {}
        T& operator[](size_t col) {
            if (col >= _matrix.nCols) {
                std::cerr << "Col : index out of bounds... Exiting" << std::endl;
                exit(EXIT_FAILURE);
            }
            return _matrix.data.coeffRef(_row, col);
        };
    };

public:
    Matrix(const unsigned long &rows, const unsigned long &cols = 1) :
            data(Eigen::SparseMatrix<T, Eigen::ColMajor, long> (
                    std::max(rows, (u_long) 1), std::max(cols, (u_long) 1))),
            nRows(rows),
            nCols(cols),
            trans(false)
    {}
    Matrix(const std::initializer_list<const std::initializer_list<T>> list) :
            data(
                    Eigen::SparseMatrix<T, Eigen::ColMajor, long>(
                            list.size() < 2 ? list.begin()->size() : list.size(),
                            list.size() < 2 ? list.size() : list.begin()->size())
            ),
            nRows((u_long) list.size() < 2 ? list.begin()->size() : list.size()),
            nCols((u_long) list.size() < 2 ? list.size() : list.begin()->size()),
            trans(false)
    {
        std::vector<triplet> tripletList(nRows * nCols);
        int i = 0; int j = 0;
        for (auto innerList : list) {
            for (auto val : innerList) {
                tripletList[(i * nCols) + j] = triplet(i, j, val);
                j++;
            }
            i++;
            j = 0;
        }
        data.setFromTriplets(tripletList.begin(), tripletList.end());
    }
    Matrix(const Matrix &matrix) :
        data(matrix.data),
        nRows(matrix.nRows),
        nCols(matrix.nCols),
        trans(matrix.trans)
    {}
    Matrix(const Eigen::SparseMatrix<T> &matrix) :
            data(matrix),
            nRows((u_long) matrix.rows()),
            nCols((u_long) matrix.cols()),
            trans(false)
    {}
    Matrix() = default;
    ~Matrix() = default;

    bool operator==(const Matrix &matrix) const;
    auto operator()() const -> decltype(data);
    auto operator[](const size_t &row) -> decltype(RowProxy{*this, row});
    Matrix<T> operator*(const Matrix<T> &matrix) const;
    Matrix<T> operator-(const Matrix<T> &matrix) const;
    Matrix<T> operator+(const Matrix<T> &matrix) const;
    void operator*=(const float &scalar);
    void operator-=(const Matrix<T> &matrix);
    void operator+=(const Matrix<T> &matrix);
    auto at(const uint16_t &index) -> decltype(data.coeffRef(index, 0));
    void identity();
    void transpose();
    void invert();
    bool canInvert();
    auto numRows() const -> decltype(nRows);
    auto numCols() const -> decltype(nCols);
    void resetMatrix();
    void zeroMatrix();

    // For testing purposes only
    void print();


};

template <class T>
bool Matrix<T>::operator==(const Matrix &matrix) const {
    return data.isApprox(matrix.data, 1E-3);
}

template <class T>
auto Matrix<T>::operator()() const -> decltype(data) {
    return data;
}

template <class T>
auto Matrix<T>::operator[](const size_t &row) -> decltype(RowProxy{*this, row}) {
    if (row >= nRows) {
        std::cerr << "Row : index out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    }
    return RowProxy{*this, row};
}

template <class T>
Matrix<T> Matrix<T>::operator*(const Matrix<T> &matrix) const {
    if (nCols != matrix.nRows) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>((data * matrix.data).pruned(1E-3));
}

template <class T>
Matrix<T> Matrix<T>::operator-(const Matrix<T> &matrix) const {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>(data - matrix.data);
}

template <class T>
Matrix<T> Matrix<T>::operator+(const Matrix<T> &matrix) const {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    return Matrix<T>(data + matrix.data);
}

template <class T>
void Matrix<T>::operator*=(const float &scalar) {
    data *= scalar;
}

template <class T>
void Matrix<T>::operator-=(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    data -= matrix.data;
}

template <class T>
void Matrix<T>::operator+=(const Matrix<T> &matrix) {
    if (nRows != matrix.nRows && nCols != matrix.nCols) {
        std::cerr << "Cannot multiply given matrices." << std::endl;
        std::cerr << "dims : A(" << nRows << ", " << nCols << ")" << std::endl;
        std::cerr << "dims : B(" << matrix.nRows << ", " << matrix.nCols << ")" << std::endl;
        exit(EXIT_FAILURE);
    }
    data += matrix.data;
}

// To be used exclusively with vectors
template <class T>
auto Matrix<T>::at(const uint16_t &index) -> decltype(data.coeffRef(index, 0)) {
    if (index >= (nRows * nCols)) {
        std::cout << "Vector : index out of bounds... Exiting" << std::endl;
        exit(EXIT_FAILURE);
    }
    return data.coeffRef(index, 0);
}

template <class T>
void Matrix<T>::identity() {
    data.setIdentity();
}

template <class T>
void Matrix<T>::transpose() {
    trans = !trans;
    std::swap(nRows, nCols);
    data = Eigen::SparseMatrix<T>(data.transpose());
}

template <class T>
void Matrix<T>::invert() {
    if (!data.nonZeros()) {
        return;
    }
//    if (canInvert()) {
        Eigen::SparseQR<Eigen::SparseMatrix<T, Eigen::ColMajor, long>, Eigen::COLAMDOrdering<long>> solver;
        solver.compute(data);
        Eigen::SparseMatrix<T, Eigen::ColMajor, long> I(nRows, nCols);
        I.setIdentity();
        data = solver.solve(I);
//    }
}

template <class T>
bool Matrix<T>::canInvert() {
    Eigen::SparseLU<Eigen::SparseMatrix<T, Eigen::ColMajor, long>, Eigen::COLAMDOrdering<long>> solver;
    solver.analyzePattern(data);
    solver.factorize(data);
    float val = solver.logAbsDeterminant();

    if (std::isinf(val)) {
        std::cerr << "Matrix not invertible. Det: " <<  val << std::endl;
        exit(EXIT_FAILURE);
    }
    return true;
}

template <class T>
auto Matrix<T>::numRows() const -> decltype(nRows){
    return nRows;
}

template <class T>
auto Matrix<T>::numCols() const -> decltype(nCols){
    return nCols;
}

template <class T>
void Matrix<T>::print() {
    std::cout << Eigen::Matrix<T, -1, -1>(data) << std::endl;
}

template <class T>
void Matrix<T>::resetMatrix() {
    if (trans) {
        trans = !trans;
        std::swap(nRows, nCols);
    }
    data = Eigen::SparseMatrix<T, Eigen::ColMajor, long>(nRows, nCols);
}

template <class T>
void Matrix<T>::zeroMatrix() {
    data.setZero();
}

#endif //MULTIAGENTSLAM_MATRIX_H
