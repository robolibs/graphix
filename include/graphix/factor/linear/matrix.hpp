#pragma once

#include <cmath>
#include <stdexcept>
#include <vector>

namespace graphix::factor {

    /**
     * @brief Simple dense matrix class for small optimization problems
     *
     * This is a minimal implementation to avoid external dependencies.
     * For large problems, consider using Eigen or similar.
     */
    class Matrix {
      public:
        /**
         * @brief Construct matrix with given dimensions
         */
        Matrix(size_t rows, size_t cols) : rows_(rows), cols_(cols), data_(rows * cols, 0.0) {}

        /**
         * @brief Construct from initializer list
         */
        Matrix(size_t rows, size_t cols, const std::vector<double> &data) : rows_(rows), cols_(cols), data_(data) {
            if (data_.size() != rows_ * cols_) {
                throw std::invalid_argument("Data size doesn't match matrix dimensions");
            }
        }

        /**
         * @brief Element access (non-const)
         */
        double &operator()(size_t i, size_t j) { return data_[i * cols_ + j]; }

        /**
         * @brief Element access (const)
         */
        double operator()(size_t i, size_t j) const { return data_[i * cols_ + j]; }

        /**
         * @brief Get number of rows
         */
        size_t rows() const { return rows_; }

        /**
         * @brief Get number of columns
         */
        size_t cols() const { return cols_; }

        /**
         * @brief Matrix transpose
         */
        Matrix transpose() const {
            Matrix result(cols_, rows_);
            for (size_t i = 0; i < rows_; ++i) {
                for (size_t j = 0; j < cols_; ++j) {
                    result(j, i) = (*this)(i, j);
                }
            }
            return result;
        }

        /**
         * @brief Matrix-matrix multiplication
         */
        Matrix operator*(const Matrix &other) const {
            if (cols_ != other.rows_) {
                throw std::invalid_argument("Matrix dimensions incompatible for multiplication");
            }

            Matrix result(rows_, other.cols_);
            for (size_t i = 0; i < rows_; ++i) {
                for (size_t j = 0; j < other.cols_; ++j) {
                    double sum = 0.0;
                    for (size_t k = 0; k < cols_; ++k) {
                        sum += (*this)(i, k) * other(k, j);
                    }
                    result(i, j) = sum;
                }
            }
            return result;
        }

        /**
         * @brief Matrix-vector multiplication (vector as column matrix)
         */
        std::vector<double> operator*(const std::vector<double> &vec) const {
            if (cols_ != vec.size()) {
                throw std::invalid_argument("Matrix-vector dimensions incompatible");
            }

            std::vector<double> result(rows_, 0.0);
            for (size_t i = 0; i < rows_; ++i) {
                for (size_t j = 0; j < cols_; ++j) {
                    result[i] += (*this)(i, j) * vec[j];
                }
            }
            return result;
        }

        /**
         * @brief Matrix addition
         */
        Matrix operator+(const Matrix &other) const {
            if (rows_ != other.rows_ || cols_ != other.cols_) {
                throw std::invalid_argument("Matrix dimensions must match for addition");
            }

            Matrix result(rows_, cols_);
            for (size_t i = 0; i < data_.size(); ++i) {
                result.data_[i] = data_[i] + other.data_[i];
            }
            return result;
        }

        /**
         * @brief Matrix subtraction
         */
        Matrix operator-(const Matrix &other) const {
            if (rows_ != other.rows_ || cols_ != other.cols_) {
                throw std::invalid_argument("Matrix dimensions must match for subtraction");
            }

            Matrix result(rows_, cols_);
            for (size_t i = 0; i < data_.size(); ++i) {
                result.data_[i] = data_[i] - other.data_[i];
            }
            return result;
        }

        /**
         * @brief Scalar multiplication
         */
        Matrix operator*(double scalar) const {
            Matrix result(rows_, cols_);
            for (size_t i = 0; i < data_.size(); ++i) {
                result.data_[i] = data_[i] * scalar;
            }
            return result;
        }

        /**
         * @brief Add scalar times identity matrix (useful for LM damping)
         */
        void add_diagonal(double value) {
            if (rows_ != cols_) {
                throw std::invalid_argument("Can only add to diagonal of square matrix");
            }
            for (size_t i = 0; i < rows_; ++i) {
                (*this)(i, i) += value;
            }
        }

        /**
         * @brief Solve Ax = b using Cholesky decomposition
         * Assumes A is symmetric positive definite
         */
        std::vector<double> solve_cholesky(const std::vector<double> &b) const;

        /**
         * @brief Get raw data
         */
        const std::vector<double> &data() const { return data_; }

        /**
         * @brief Frobenius norm
         */
        double norm() const {
            double sum = 0.0;
            for (double val : data_) {
                sum += val * val;
            }
            return std::sqrt(sum);
        }

      private:
        size_t rows_;
        size_t cols_;
        std::vector<double> data_;
    };

    /**
     * @brief Vector operations
     */
    namespace vector {
        inline double dot(const std::vector<double> &a, const std::vector<double> &b) {
            if (a.size() != b.size()) {
                throw std::invalid_argument("Vector sizes must match for dot product");
            }
            double result = 0.0;
            for (size_t i = 0; i < a.size(); ++i) {
                result += a[i] * b[i];
            }
            return result;
        }

        inline double norm(const std::vector<double> &v) { return std::sqrt(dot(v, v)); }

        inline std::vector<double> operator+(const std::vector<double> &a, const std::vector<double> &b) {
            if (a.size() != b.size()) {
                throw std::invalid_argument("Vector sizes must match for addition");
            }
            std::vector<double> result(a.size());
            for (size_t i = 0; i < a.size(); ++i) {
                result[i] = a[i] + b[i];
            }
            return result;
        }

        inline std::vector<double> operator-(const std::vector<double> &a, const std::vector<double> &b) {
            if (a.size() != b.size()) {
                throw std::invalid_argument("Vector sizes must match for subtraction");
            }
            std::vector<double> result(a.size());
            for (size_t i = 0; i < a.size(); ++i) {
                result[i] = a[i] - b[i];
            }
            return result;
        }

        inline std::vector<double> operator*(const std::vector<double> &v, double scalar) {
            std::vector<double> result(v.size());
            for (size_t i = 0; i < v.size(); ++i) {
                result[i] = v[i] * scalar;
            }
            return result;
        }

        inline std::vector<double> operator*(double scalar, const std::vector<double> &v) { return v * scalar; }
    } // namespace vector

} // namespace graphix::factor
