// mat.hpp — 고정 크기 행렬 (Eigen 없이). IMM-EKF 가 쓰는 6×6 이하 연산만.
// 의존성 0 이라 나중에 FC(ChibiOS) 나 MCU 로 그대로 옮길 수 있다. 연산 순서를 numpy 와 같게 두어
// 파이썬 오라클과 1e-12 급으로 맞는다 (test_fixes 'C++ 코어:' 차등 검사, test_closed_loop --core cpp).
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace mars {

template <std::size_t R, std::size_t C>
struct Mat {
    std::array<double, R * C> a{};

    static constexpr std::size_t rows = R;
    static constexpr std::size_t cols = C;

    double& operator()(std::size_t i, std::size_t j) { return a[i * C + j]; }
    double operator()(std::size_t i, std::size_t j) const { return a[i * C + j]; }

    static Mat zero() { return Mat{}; }
    static Mat identity() {
        static_assert(R == C, "identity needs square");
        Mat m;
        for (std::size_t i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    Mat<C, R> transpose() const {
        Mat<C, R> t;
        for (std::size_t i = 0; i < R; ++i)
            for (std::size_t j = 0; j < C; ++j) t(j, i) = (*this)(i, j);
        return t;
    }

    bool finite() const {
        for (double v : a)
            if (!std::isfinite(v)) return false;
        return true;
    }

    double trace() const {
        static_assert(R == C, "trace needs square");
        double s = 0.0;
        for (std::size_t i = 0; i < R; ++i) s += (*this)(i, i);
        return s;
    }
};

template <std::size_t N>
using Vec = Mat<N, 1>;

template <std::size_t R, std::size_t C>
Mat<R, C> operator+(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> o;
    for (std::size_t i = 0; i < R * C; ++i) o.a[i] = x.a[i] + y.a[i];
    return o;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator-(const Mat<R, C>& x, const Mat<R, C>& y) {
    Mat<R, C> o;
    for (std::size_t i = 0; i < R * C; ++i) o.a[i] = x.a[i] - y.a[i];
    return o;
}

template <std::size_t R, std::size_t C>
Mat<R, C> operator*(double s, const Mat<R, C>& x) {
    Mat<R, C> o;
    for (std::size_t i = 0; i < R * C; ++i) o.a[i] = s * x.a[i];
    return o;
}

template <std::size_t R, std::size_t K, std::size_t C>
Mat<R, C> operator*(const Mat<R, K>& x, const Mat<K, C>& y) {
    Mat<R, C> o;
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j) {
            double s = 0.0;
            for (std::size_t k = 0; k < K; ++k) s += x(i, k) * y(k, j);
            o(i, j) = s;
        }
    return o;
}

template <std::size_t N>
double dot(const Vec<N>& x, const Vec<N>& y) {
    double s = 0.0;
    for (std::size_t i = 0; i < N; ++i) s += x.a[i] * y.a[i];
    return s;
}

template <std::size_t N>
Mat<N, N> outer(const Vec<N>& x, const Vec<N>& y) {
    Mat<N, N> o;
    for (std::size_t i = 0; i < N; ++i)
        for (std::size_t j = 0; j < N; ++j) o(i, j) = x.a[i] * y.a[j];
    return o;
}

// 가우스-조던(부분 피벗) 역행렬. 특이하면 false (numpy LinAlgError 에 대응).
template <std::size_t N>
bool inverse(const Mat<N, N>& m, Mat<N, N>& out) {
    Mat<N, N> a = m;
    Mat<N, N> inv = Mat<N, N>::identity();
    for (std::size_t col = 0; col < N; ++col) {
        std::size_t piv = col;
        double best = std::fabs(a(col, col));
        for (std::size_t r = col + 1; r < N; ++r) {
            const double v = std::fabs(a(r, col));
            if (v > best) { best = v; piv = r; }
        }
        if (!(best > 0.0) || !std::isfinite(best)) return false;
        if (piv != col)
            for (std::size_t j = 0; j < N; ++j) {
                std::swap(a(col, j), a(piv, j));
                std::swap(inv(col, j), inv(piv, j));
            }
        const double d = a(col, col);
        for (std::size_t j = 0; j < N; ++j) { a(col, j) /= d; inv(col, j) /= d; }
        for (std::size_t r = 0; r < N; ++r) {
            if (r == col) continue;
            const double f = a(r, col);
            if (f == 0.0) continue;
            for (std::size_t j = 0; j < N; ++j) { a(r, j) -= f * a(col, j); inv(r, j) -= f * inv(col, j); }
        }
    }
    out = inv;
    return true;
}

// LU(부분 피벗) 행렬식.
template <std::size_t N>
double determinant(const Mat<N, N>& m) {
    Mat<N, N> a = m;
    double det = 1.0;
    for (std::size_t col = 0; col < N; ++col) {
        std::size_t piv = col;
        double best = std::fabs(a(col, col));
        for (std::size_t r = col + 1; r < N; ++r) {
            const double v = std::fabs(a(r, col));
            if (v > best) { best = v; piv = r; }
        }
        if (!(best > 0.0)) return 0.0;
        if (piv != col) {
            for (std::size_t j = 0; j < N; ++j) std::swap(a(col, j), a(piv, j));
            det = -det;
        }
        det *= a(col, col);
        for (std::size_t r = col + 1; r < N; ++r) {
            const double f = a(r, col) / a(col, col);
            for (std::size_t j = col; j < N; ++j) a(r, j) -= f * a(col, j);
        }
    }
    return det;
}

inline double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace mars
