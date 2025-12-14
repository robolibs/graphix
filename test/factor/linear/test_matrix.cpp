#include "graphix/factor/linear/matrix.hpp"
#include <doctest/doctest.h>

using namespace graphix::factor;

TEST_CASE("Matrix - construction and dimensions") {
    SUBCASE("Default construction") {
        Matrix m(3, 2);
        CHECK(m.rows() == 3);
        CHECK(m.cols() == 2);
        // All elements should be zero-initialized
        for (size_t i = 0; i < 3; ++i) {
            for (size_t j = 0; j < 2; ++j) {
                CHECK(m(i, j) == 0.0);
            }
        }
    }

    SUBCASE("Construction from data") {
        std::vector<double> data = {1, 2, 3, 4, 5, 6};
        Matrix m(2, 3, data);
        CHECK(m.rows() == 2);
        CHECK(m.cols() == 3);
        CHECK(m(0, 0) == 1);
        CHECK(m(0, 1) == 2);
        CHECK(m(0, 2) == 3);
        CHECK(m(1, 0) == 4);
        CHECK(m(1, 1) == 5);
        CHECK(m(1, 2) == 6);
    }

    SUBCASE("Construction with wrong data size") {
        std::vector<double> data = {1, 2, 3};
        CHECK_THROWS_AS(Matrix(2, 3, data), std::invalid_argument);
    }
}

TEST_CASE("Matrix - element access and modification") {
    Matrix m(2, 2);

    m(0, 0) = 1.5;
    m(0, 1) = 2.5;
    m(1, 0) = 3.5;
    m(1, 1) = 4.5;

    CHECK(m(0, 0) == 1.5);
    CHECK(m(0, 1) == 2.5);
    CHECK(m(1, 0) == 3.5);
    CHECK(m(1, 1) == 4.5);
}

TEST_CASE("Matrix - transpose") {
    SUBCASE("Rectangular matrix") {
        Matrix m(2, 3);
        m(0, 0) = 1.0;
        m(0, 1) = 2.0;
        m(0, 2) = 3.0;
        m(1, 0) = 4.0;
        m(1, 1) = 5.0;
        m(1, 2) = 6.0;

        Matrix mt = m.transpose();
        CHECK(mt.rows() == 3);
        CHECK(mt.cols() == 2);
        CHECK(mt(0, 0) == 1.0);
        CHECK(mt(1, 0) == 2.0);
        CHECK(mt(2, 0) == 3.0);
        CHECK(mt(0, 1) == 4.0);
        CHECK(mt(1, 1) == 5.0);
        CHECK(mt(2, 1) == 6.0);
    }

    SUBCASE("Square matrix") {
        Matrix m(2, 2);
        m(0, 0) = 1.0;
        m(0, 1) = 2.0;
        m(1, 0) = 3.0;
        m(1, 1) = 4.0;

        Matrix mt = m.transpose();
        CHECK(mt(0, 0) == 1.0);
        CHECK(mt(0, 1) == 3.0);
        CHECK(mt(1, 0) == 2.0);
        CHECK(mt(1, 1) == 4.0);
    }
}

TEST_CASE("Matrix - matrix multiplication") {
    SUBCASE("Basic multiplication") {
        Matrix a(2, 3);
        a(0, 0) = 1;
        a(0, 1) = 2;
        a(0, 2) = 3;
        a(1, 0) = 4;
        a(1, 1) = 5;
        a(1, 2) = 6;

        Matrix b(3, 2);
        b(0, 0) = 7;
        b(0, 1) = 8;
        b(1, 0) = 9;
        b(1, 1) = 10;
        b(2, 0) = 11;
        b(2, 1) = 12;

        Matrix c = a * b;
        CHECK(c.rows() == 2);
        CHECK(c.cols() == 2);
        CHECK(c(0, 0) == doctest::Approx(58));  // 1*7 + 2*9 + 3*11
        CHECK(c(0, 1) == doctest::Approx(64));  // 1*8 + 2*10 + 3*12
        CHECK(c(1, 0) == doctest::Approx(139)); // 4*7 + 5*9 + 6*11
        CHECK(c(1, 1) == doctest::Approx(154)); // 4*8 + 5*10 + 6*12
    }

    SUBCASE("Identity multiplication") {
        Matrix I(2, 2);
        I(0, 0) = 1;
        I(1, 1) = 1;

        Matrix m(2, 2);
        m(0, 0) = 3;
        m(0, 1) = 4;
        m(1, 0) = 5;
        m(1, 1) = 6;

        Matrix result = I * m;
        CHECK(result(0, 0) == 3);
        CHECK(result(0, 1) == 4);
        CHECK(result(1, 0) == 5);
        CHECK(result(1, 1) == 6);
    }

    SUBCASE("Incompatible dimensions") {
        Matrix a(2, 3);
        Matrix b(2, 2);
        CHECK_THROWS_AS(a * b, std::invalid_argument);
    }
}

TEST_CASE("Matrix - matrix-vector multiplication") {
    SUBCASE("Basic operation") {
        Matrix A(2, 3);
        A(0, 0) = 1;
        A(0, 1) = 2;
        A(0, 2) = 3;
        A(1, 0) = 4;
        A(1, 1) = 5;
        A(1, 2) = 6;

        std::vector<double> x = {1, 2, 3};
        std::vector<double> y = A * x;

        CHECK(y.size() == 2);
        CHECK(y[0] == doctest::Approx(14)); // 1*1 + 2*2 + 3*3
        CHECK(y[1] == doctest::Approx(32)); // 4*1 + 5*2 + 6*3
    }

    SUBCASE("Identity matrix") {
        Matrix I(3, 3);
        I(0, 0) = 1;
        I(1, 1) = 1;
        I(2, 2) = 1;

        std::vector<double> v = {2, 3, 4};
        std::vector<double> result = I * v;

        CHECK(result[0] == 2);
        CHECK(result[1] == 3);
        CHECK(result[2] == 4);
    }

    SUBCASE("Incompatible dimensions") {
        Matrix A(2, 3);
        std::vector<double> v = {1, 2};
        CHECK_THROWS_AS(A * v, std::invalid_argument);
    }
}

TEST_CASE("Matrix - addition") {
    SUBCASE("Basic addition") {
        Matrix a(2, 2);
        a(0, 0) = 1;
        a(0, 1) = 2;
        a(1, 0) = 3;
        a(1, 1) = 4;

        Matrix b(2, 2);
        b(0, 0) = 5;
        b(0, 1) = 6;
        b(1, 0) = 7;
        b(1, 1) = 8;

        Matrix c = a + b;
        CHECK(c(0, 0) == 6);
        CHECK(c(0, 1) == 8);
        CHECK(c(1, 0) == 10);
        CHECK(c(1, 1) == 12);
    }

    SUBCASE("Incompatible dimensions") {
        Matrix a(2, 2);
        Matrix b(2, 3);
        CHECK_THROWS_AS(a + b, std::invalid_argument);
    }
}

TEST_CASE("Matrix - subtraction") {
    SUBCASE("Basic subtraction") {
        Matrix a(2, 2);
        a(0, 0) = 5;
        a(0, 1) = 6;
        a(1, 0) = 7;
        a(1, 1) = 8;

        Matrix b(2, 2);
        b(0, 0) = 1;
        b(0, 1) = 2;
        b(1, 0) = 3;
        b(1, 1) = 4;

        Matrix c = a - b;
        CHECK(c(0, 0) == 4);
        CHECK(c(0, 1) == 4);
        CHECK(c(1, 0) == 4);
        CHECK(c(1, 1) == 4);
    }

    SUBCASE("Incompatible dimensions") {
        Matrix a(2, 2);
        Matrix b(3, 2);
        CHECK_THROWS_AS(a - b, std::invalid_argument);
    }
}

TEST_CASE("Matrix - scalar multiplication") {
    Matrix m(2, 2);
    m(0, 0) = 1;
    m(0, 1) = 2;
    m(1, 0) = 3;
    m(1, 1) = 4;

    Matrix result = m * 2.0;
    CHECK(result(0, 0) == 2);
    CHECK(result(0, 1) == 4);
    CHECK(result(1, 0) == 6);
    CHECK(result(1, 1) == 8);
}

TEST_CASE("Matrix - add diagonal") {
    SUBCASE("Square matrix") {
        Matrix A(3, 3);
        A(0, 0) = 1;
        A(1, 1) = 2;
        A(2, 2) = 3;

        A.add_diagonal(10);

        CHECK(A(0, 0) == 11);
        CHECK(A(1, 1) == 12);
        CHECK(A(2, 2) == 13);
    }

    SUBCASE("Non-square matrix throws") {
        Matrix A(2, 3);
        CHECK_THROWS_AS(A.add_diagonal(5), std::invalid_argument);
    }
}

TEST_CASE("Matrix - Frobenius norm") {
    Matrix m(2, 2);
    m(0, 0) = 1;
    m(0, 1) = 2;
    m(1, 0) = 3;
    m(1, 1) = 4;

    // sqrt(1 + 4 + 9 + 16) = sqrt(30)
    CHECK(m.norm() == doctest::Approx(std::sqrt(30)));
}

TEST_CASE("Matrix - Cholesky solver - simple 2x2") {
    // Solve [4 2] [x1]   [2]
    //       [2 3] [x2] = [1]
    // Solution: x1 = 0.5, x2 = 0

    Matrix A(2, 2);
    A(0, 0) = 4;
    A(0, 1) = 2;
    A(1, 0) = 2;
    A(1, 1) = 3;

    std::vector<double> b = {2, 1};
    std::vector<double> x = A.solve_cholesky(b);

    CHECK(x.size() == 2);
    CHECK(x[0] == doctest::Approx(0.5).epsilon(0.01));
    CHECK(x[1] == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("Matrix - Cholesky solver - 3x3 identity") {
    Matrix I(3, 3);
    I(0, 0) = 1;
    I(1, 1) = 1;
    I(2, 2) = 1;

    std::vector<double> b = {1, 2, 3};
    std::vector<double> x = I.solve_cholesky(b);

    CHECK(x[0] == doctest::Approx(1));
    CHECK(x[1] == doctest::Approx(2));
    CHECK(x[2] == doctest::Approx(3));
}

TEST_CASE("Matrix - Cholesky solver - larger system") {
    // Test a larger symmetric positive definite system
    Matrix A(3, 3);
    A(0, 0) = 4;
    A(0, 1) = 2;
    A(0, 2) = 1;
    A(1, 0) = 2;
    A(1, 1) = 5;
    A(1, 2) = 2;
    A(2, 0) = 1;
    A(2, 1) = 2;
    A(2, 2) = 6;

    std::vector<double> b = {1, 2, 3};
    std::vector<double> x = A.solve_cholesky(b);

    // Verify Ax = b
    std::vector<double> result = A * x;
    CHECK(result[0] == doctest::Approx(b[0]).epsilon(0.01));
    CHECK(result[1] == doctest::Approx(b[1]).epsilon(0.01));
    CHECK(result[2] == doctest::Approx(b[2]).epsilon(0.01));
}

TEST_CASE("Matrix - Cholesky fails on non-positive-definite") {
    SUBCASE("Negative diagonal element") {
        Matrix A(2, 2);
        A(0, 0) = -1;
        A(0, 1) = 0;
        A(1, 0) = 0;
        A(1, 1) = 1;

        std::vector<double> b = {1, 1};
        CHECK_THROWS_AS(A.solve_cholesky(b), std::runtime_error);
    }

    SUBCASE("Zero diagonal") {
        Matrix A(2, 2);
        A(0, 0) = 0;
        A(0, 1) = 0;
        A(1, 0) = 0;
        A(1, 1) = 1;

        std::vector<double> b = {1, 1};
        CHECK_THROWS_AS(A.solve_cholesky(b), std::runtime_error);
    }
}

TEST_CASE("Vector operations - dot product") {
    using namespace graphix::factor::vector;

    SUBCASE("Basic dot product") {
        std::vector<double> a = {1, 2, 3};
        std::vector<double> b = {4, 5, 6};

        double result = dot(a, b);
        CHECK(result == doctest::Approx(32)); // 1*4 + 2*5 + 3*6
    }

    SUBCASE("Orthogonal vectors") {
        std::vector<double> a = {1, 0, 0};
        std::vector<double> b = {0, 1, 0};

        CHECK(dot(a, b) == 0);
    }

    SUBCASE("Mismatched sizes") {
        std::vector<double> a = {1, 2};
        std::vector<double> b = {1, 2, 3};

        CHECK_THROWS_AS(dot(a, b), std::invalid_argument);
    }
}

TEST_CASE("Vector operations - norm") {
    using namespace graphix::factor::vector;

    SUBCASE("3-4-5 triangle") {
        std::vector<double> v = {3, 4};
        CHECK(norm(v) == doctest::Approx(5)); // sqrt(9 + 16)
    }

    SUBCASE("Unit vector") {
        std::vector<double> v = {1, 0, 0};
        CHECK(norm(v) == doctest::Approx(1));
    }

    SUBCASE("Zero vector") {
        std::vector<double> v = {0, 0, 0};
        CHECK(norm(v) == 0);
    }
}

TEST_CASE("Vector operations - addition") {
    using namespace graphix::factor::vector;

    SUBCASE("Basic addition") {
        std::vector<double> a = {1, 2, 3};
        std::vector<double> b = {4, 5, 6};
        std::vector<double> c = a + b;

        CHECK(c.size() == 3);
        CHECK(c[0] == 5);
        CHECK(c[1] == 7);
        CHECK(c[2] == 9);
    }

    SUBCASE("Mismatched sizes") {
        std::vector<double> a = {1, 2};
        std::vector<double> b = {1, 2, 3};

        CHECK_THROWS_AS(a + b, std::invalid_argument);
    }
}

TEST_CASE("Vector operations - subtraction") {
    using namespace graphix::factor::vector;

    SUBCASE("Basic subtraction") {
        std::vector<double> a = {5, 7, 9};
        std::vector<double> b = {1, 2, 3};
        std::vector<double> c = a - b;

        CHECK(c[0] == 4);
        CHECK(c[1] == 5);
        CHECK(c[2] == 6);
    }

    SUBCASE("Mismatched sizes") {
        std::vector<double> a = {1, 2, 3};
        std::vector<double> b = {1, 2};

        CHECK_THROWS_AS(a - b, std::invalid_argument);
    }
}

TEST_CASE("Vector operations - scalar multiplication") {
    using namespace graphix::factor::vector;

    SUBCASE("Vector * scalar") {
        std::vector<double> v = {1, 2, 3};
        std::vector<double> result = v * 2.0;

        CHECK(result[0] == 2);
        CHECK(result[1] == 4);
        CHECK(result[2] == 6);
    }

    SUBCASE("Scalar * vector") {
        std::vector<double> v = {1, 2, 3};
        std::vector<double> result = 3.0 * v;

        CHECK(result[0] == 3);
        CHECK(result[1] == 6);
        CHECK(result[2] == 9);
    }
}
