#pragma once
#include <cmath>

namespace raytracer::engine {

/**
 * A vector in R^3, represented by its three Cartesian components.
 */
struct Vec3 {
    double x, y, z;

    /**
     * Vector subtraction: this - U, component-wise.
     * @param U the vector to subtract.
     * @return a new Vec3.
     */
    Vec3 operator-(const Vec3& U) const {
        return Vec3{x - U.x, y - U.y, z - U.z};
    }

    /**
     * Vector addition: this + U, component-wise.
     * @param U the vector to add.
     * @return a new Vec3.
     */
    Vec3 operator+(const Vec3& U) const {
        return Vec3{x + U.x, y + U.y, z + U.z};
    }

    /**
     * Dot product (inner product): this . U = sum_i (this_i * U_i).
     * Returns a scalar, not a vector.
     * @param U the second vector of the product.
     * @return a double.
     */
    double dot(const Vec3& U) const {
        return x * U.x + y * U.y + z * U.z;
    }

    /**
     * Scalar multiplication: this * t.
     * @param t the scalar factor.
     * @return a new Vec3, each component scaled by t.
     */
    Vec3 operator*(double t) const {
        return Vec3{x * t, y * t, z * t};
    }

    /**
     * Squared Euclidean norm: ||this||^2 = this . this.
     * Avoids a square root when only the square is needed.
     * @return a double, always >= 0.
     */
    double length_squared() const {
        return dot(*this);
    }

    /**
     * Euclidean norm: ||this|| = sqrt(this . this).
     * @return a double, always >= 0.
     */
    double length() const {
        return std::sqrt(length_squared());
    }

    /**
     * Unit-length vector in the same direction as this.
     * Precondition: ||this|| != 0 (not checked here).
     * @return a new Vec3 of norm 1.
     */
    Vec3 normalized() const {
        return *this * (1.0 / length());
    }
};

}  // namespace raytracer::engine