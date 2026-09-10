#include "radar/linalg.hpp"

#include <gtest/gtest.h>

using namespace radar;

TEST(Linalg, Mat4IdentityMultiplyIsNoOp) {
    Mat4 id = mat4_identity();
    Vec4 v{{1.0, 2.0, 3.0, 4.0}};
    Vec4 out = mat4_mul_vec4(id, v);
    for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(out.v[i], v.v[i]);
}

TEST(Linalg, Mat4TransposeTwiceIsIdentity) {
    Mat4 a;
    int c = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) a.m[i][j] = static_cast<double>(++c);
    Mat4 tt = mat4_transpose(mat4_transpose(a));
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) EXPECT_DOUBLE_EQ(tt.m[i][j], a.m[i][j]);
}

TEST(Linalg, Mat2InverseRoundTrip) {
    Mat2 a;
    a.m[0][0] = 4.0;
    a.m[0][1] = 7.0;
    a.m[1][0] = 2.0;
    a.m[1][1] = 6.0;
    Mat2 inv = mat2_inverse(a);
    // a * inv should be the 2x2 identity.
    Mat2 prod;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            double s = 0.0;
            for (int k = 0; k < 2; ++k) s += a.m[i][k] * inv.m[k][j];
            prod.m[i][j] = s;
        }
    EXPECT_NEAR(prod.m[0][0], 1.0, 1e-9);
    EXPECT_NEAR(prod.m[0][1], 0.0, 1e-9);
    EXPECT_NEAR(prod.m[1][0], 0.0, 1e-9);
    EXPECT_NEAR(prod.m[1][1], 1.0, 1e-9);
}

TEST(Linalg, Mat4AddIsElementwise) {
    Mat4 a, b;
    a.m[1][2] = 3.0;
    b.m[1][2] = 4.0;
    Mat4 c = mat4_add(a, b);
    EXPECT_DOUBLE_EQ(c.m[1][2], 7.0);
}
