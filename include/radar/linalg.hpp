#pragma once
// Small fixed-size matrix/vector helpers for the constant-velocity Kalman
// filter. State dimension is 4 (x, y, vx, vy), measurement dimension is 2
// (x, y), so everything here is hand-rolled rather than pulled from a linear
// algebra library. No dynamic allocation, no exceptions.

namespace radar {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec4 {
    double v[4] = {0.0, 0.0, 0.0, 0.0};
};

struct Mat2 {
    double m[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
};

struct Mat4 {
    double m[4][4] = {};
};

// 2 rows x 4 cols, used for the measurement matrix H and H*P.
struct Mat24 {
    double m[2][4] = {};
};

// 4 rows x 2 cols, used for H^T and the Kalman gain K.
struct Mat42 {
    double m[4][2] = {};
};

inline Mat4 mat4_identity() {
    Mat4 out;
    for (int i = 0; i < 4; ++i) out.m[i][i] = 1.0;
    return out;
}

inline Mat4 mat4_mul(const Mat4& a, const Mat4& b) {
    Mat4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += a.m[i][k] * b.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

inline Mat4 mat4_add(const Mat4& a, const Mat4& b) {
    Mat4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out.m[i][j] = a.m[i][j] + b.m[i][j];
    return out;
}

inline Mat4 mat4_transpose(const Mat4& a) {
    Mat4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) out.m[j][i] = a.m[i][j];
    return out;
}

inline Vec4 mat4_mul_vec4(const Mat4& a, const Vec4& v) {
    Vec4 out;
    for (int i = 0; i < 4; ++i) {
        double s = 0.0;
        for (int k = 0; k < 4; ++k) s += a.m[i][k] * v.v[k];
        out.v[i] = s;
    }
    return out;
}

inline Vec4 vec4_add(const Vec4& a, const Vec4& b) {
    Vec4 out;
    for (int i = 0; i < 4; ++i) out.v[i] = a.v[i] + b.v[i];
    return out;
}

// H * P : (2x4) * (4x4) = 2x4
inline Mat24 mat24_mul_mat4(const Mat24& h, const Mat4& p) {
    Mat24 out;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += h.m[i][k] * p.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

// (H*P) * H^T : (2x4) * (4x2) = 2x2
inline Mat2 mat24_mul_mat42(const Mat24& a, const Mat42& b) {
    Mat2 out;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += a.m[i][k] * b.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

// P * H^T : (4x4) * (4x2) = 4x2
inline Mat42 mat4_mul_mat42(const Mat4& p, const Mat42& ht) {
    Mat42 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += p.m[i][k] * ht.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

inline Mat42 mat24_transpose(const Mat24& h) {
    Mat42 out;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 4; ++j) out.m[j][i] = h.m[i][j];
    return out;
}

inline Mat2 mat2_add(const Mat2& a, const Mat2& b) {
    Mat2 out;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) out.m[i][j] = a.m[i][j] + b.m[i][j];
    return out;
}

// Analytic 2x2 inverse. The caller (gating/update) guarantees S is a valid
// innovation covariance (positive definite by construction: H P H^T + R with
// R positive definite), so the determinant is bounded away from zero except
// under numerical blow-up, which the covariance-inflation test in
// test_kalman.cpp exists to catch.
inline Mat2 mat2_inverse(const Mat2& a) {
    double det = a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0];
    double inv_det = (det != 0.0) ? (1.0 / det) : 0.0;
    Mat2 out;
    out.m[0][0] = a.m[1][1] * inv_det;
    out.m[0][1] = -a.m[0][1] * inv_det;
    out.m[1][0] = -a.m[1][0] * inv_det;
    out.m[1][1] = a.m[0][0] * inv_det;
    return out;
}

inline Vec2 mat2_mul_vec2(const Mat2& a, const Vec2& v) {
    Vec2 out;
    out.x = a.m[0][0] * v.x + a.m[0][1] * v.y;
    out.y = a.m[1][0] * v.x + a.m[1][1] * v.y;
    return out;
}

inline Vec2 vec2_sub(const Vec2& a, const Vec2& b) { return Vec2{a.x - b.x, a.y - b.y}; }

inline double vec2_dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }

// K = (P H^T) * S^-1 : (4x2) * (2x2) = 4x2
inline Mat42 mat42_mul_mat2(const Mat42& a, const Mat2& b) {
    Mat42 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 2; ++j) {
            double s = 0.0;
            for (int k = 0; k < 2; ++k) s += a.m[i][k] * b.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

// K * y : (4x2) * (2x1) = 4x1
inline Vec4 mat42_mul_vec2(const Mat42& a, const Vec2& v) {
    Vec4 out;
    for (int i = 0; i < 4; ++i) out.v[i] = a.m[i][0] * v.x + a.m[i][1] * v.y;
    return out;
}

// K * H : (4x2) * (2x4) = 4x4
inline Mat4 mat42_mul_mat24(const Mat42& a, const Mat24& b) {
    Mat4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 2; ++k) s += a.m[i][k] * b.m[k][j];
            out.m[i][j] = s;
        }
    return out;
}

} // namespace radar
