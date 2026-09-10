#include "radar/kalman.hpp"

namespace radar {

void KalmanCV::init(double x, double y, double vx, double vy, double q_density,
                     double measurement_sigma_m) {
    x_.v[0] = x;
    x_.v[1] = y;
    x_.v[2] = vx;
    x_.v[3] = vy;

    p_ = Mat4{};
    // Initial position uncertainty from the seeding detection's own noise;
    // velocity is unobserved on a single detection, so its prior variance
    // starts large and is worked down by subsequent updates.
    const double pos_var = measurement_sigma_m * measurement_sigma_m;
    const double vel_var = 400.0 * 400.0; // m^2/s^2, a deliberately loose prior
    p_.m[0][0] = pos_var;
    p_.m[1][1] = pos_var;
    p_.m[2][2] = vel_var;
    p_.m[3][3] = vel_var;

    q_density_ = q_density;

    r_ = Mat2{};
    r_.m[0][0] = measurement_sigma_m * measurement_sigma_m;
    r_.m[1][1] = measurement_sigma_m * measurement_sigma_m;
}

void KalmanCV::predict(double dt) {
    Mat4 f = mat4_identity();
    f.m[0][2] = dt;
    f.m[1][3] = dt;

    x_ = mat4_mul_vec4(f, x_);

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Mat4 q{};
    q.m[0][0] = q_density_ * dt4 / 4.0;
    q.m[0][2] = q_density_ * dt3 / 2.0;
    q.m[2][0] = q_density_ * dt3 / 2.0;
    q.m[2][2] = q_density_ * dt2;
    q.m[1][1] = q_density_ * dt4 / 4.0;
    q.m[1][3] = q_density_ * dt3 / 2.0;
    q.m[3][1] = q_density_ * dt3 / 2.0;
    q.m[3][3] = q_density_ * dt2;

    Mat4 fp = mat4_mul(f, p_);
    Mat4 fpft = mat4_mul(fp, mat4_transpose(f));
    p_ = mat4_add(fpft, q);
}

namespace {
Mat24 measurement_matrix() {
    Mat24 h{};
    h.m[0][0] = 1.0;
    h.m[1][1] = 1.0;
    return h;
}
} // namespace

void KalmanCV::innovation(const Detection& z, Vec2& y_out, Mat2& s_out) const {
    const Mat24 h = measurement_matrix();
    Vec2 hx{x_.v[0], x_.v[1]};
    Vec2 zv{z.x, z.y};
    y_out = vec2_sub(zv, hx);

    Mat24 hp = mat24_mul_mat4(h, p_);
    Mat42 ht = mat24_transpose(h);
    Mat2 hpht = mat24_mul_mat42(hp, ht);
    s_out = mat2_add(hpht, r_);
}

void KalmanCV::update(const Vec2& y, const Mat2& s) {
    const Mat24 h = measurement_matrix();
    Mat42 ht = mat24_transpose(h);
    Mat42 pht = mat4_mul_mat42(p_, ht);
    Mat2 s_inv = mat2_inverse(s);
    Mat42 k = mat42_mul_mat2(pht, s_inv);

    Vec4 correction = mat42_mul_vec2(k, y);
    x_ = vec4_add(x_, correction);

    Mat4 kh = mat42_mul_mat24(k, h);
    Mat4 i_minus_kh = mat4_identity();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) i_minus_kh.m[i][j] -= kh.m[i][j];
    p_ = mat4_mul(i_minus_kh, p_);
}

} // namespace radar
