#pragma once
#include <cmath>
#include "vec3.hpp"

namespace raytracer::engine {

/**
 * A 3x3 matrix, stored row-major: m[i][j] is the entry A_ij
 * (row i, column j).
 */
struct Mat3 {
    double m[3][3];

    /**
     * Matrix-vector product: this * v, i.e. (this*v)_i = sum_j (m[i][j] * v_j).
     * @param v the vector to transform.
     * @return a new Vec3.
     */
    Vec3 operator*(const Vec3& v) const {
        return Vec3{
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    /**
     * Matrix addition: this + N, component-wise.
     * @param N the matrix to add.
     * @return a new Mat3.
     */
    Mat3 operator+(const Mat3& N) const {
        Mat3 result{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[i][j] = m[i][j] + N.m[i][j];
        return result;
    }

    /**
     * Matrix subtraction: this - N, component-wise.
     * @param N the matrix to subtract.
     * @return a new Mat3.
     */
    Mat3 operator-(const Mat3& N) const {
        Mat3 result{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[i][j] = m[i][j] - N.m[i][j];
        return result;
    }

    /**
     * Scalar multiplication: this * t, component-wise.
     * @param t the scalar factor.
     * @return a new Mat3.
     */
    Mat3 operator*(double t) const {
        Mat3 result{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[i][j] = m[i][j] * t;
        return result;
    }

    /**
     * Transpose: (this^T)_ij = this_ji.
     * @return a new Mat3 with rows and columns swapped.
     */
    Mat3 transpose() const {
        Mat3 result{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.m[j][i] = m[i][j];
        return result;
    }

    /**
     * Matrix multiplication: this * N (composition of linear maps).
     * Not commutative in general: this*N != N*this.
     * @param N the matrix to multiply on the right.
     * @return a new Mat3.
     */
    Mat3 operator*(const Mat3& N) const {
        Mat3 result{};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k)
                    sum += m[i][k] * N.m[k][j];
                result.m[i][j] = sum;
            }
        return result;
    }
};

/**
 * Commutative counterpart of Mat3::operator*(double), so that t * M and
 * M * t both compile.
 * @param t the scalar factor.
 * @param M the matrix to scale.
 * @return a new Mat3.
 */
inline Mat3 operator*(double t, const Mat3& M) {
    return M * t;
}

/**
 * 3x3 identity matrix.
 * @return a new Mat3 equal to I.
 */
inline Mat3 identity3() {
    return Mat3{{
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    }};
}

/**
 * Rotation matrix about the x-axis by angle theta (radians),
 * right-handed convention.
 * @param theta rotation angle in radians.
 * @return a new Mat3.
 */
inline Mat3 rotation_x(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    return Mat3{{
        {1.0, 0.0, 0.0},
        {0.0,  c,  -s},
        {0.0,  s,   c}
    }};
}

/**
 * Rotation matrix about the y-axis by angle theta (radians),
 * right-handed convention.
 * @param theta rotation angle in radians.
 * @return a new Mat3.
 */
inline Mat3 rotation_y(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    return Mat3{{
        { c,  0.0,  s},
        {0.0, 1.0, 0.0},
        {-s,  0.0,  c}
    }};
}

/**
 * Rotation matrix about the z-axis by angle theta (radians),
 * right-handed convention.
 * @param theta rotation angle in radians.
 * @return a new Mat3.
 */
inline Mat3 rotation_z(double theta) {
    double c = std::cos(theta), s = std::sin(theta);
    return Mat3{{
        { c,  -s, 0.0},
        { s,   c, 0.0},
        {0.0, 0.0, 1.0}
    }};
}

}  // namespace raytracer::engine