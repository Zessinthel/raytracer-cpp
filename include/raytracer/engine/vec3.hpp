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
         * Scalar multiplication: this * t.
         * @param t the scalar factor.
         * @return a new Vec3, each component scaled by t.
         */
        Vec3 operator*(double t) const {
            return Vec3{x * t, y * t, z * t};
        }
    };

    /**
     * Commutative counterpart of Vec3::operator*, so that t * v and v * t
     * both compile. Delegates to the member operator to avoid duplicating
     * the componentwise logic.
     * @param t the scalar factor.
     * @param v the vector to scale.
     * @return a new Vec3, each component scaled by t.
     */
    inline Vec3 operator*(double t, const Vec3& v) {
        return v * t;
    }

    /**
     * Dot product (inner product): a . b = sum_i (a_i * b_i).
     * Returns a scalar, not a vector.
     * @param a first vector.
     * @param b second vector.
     * @return a double.
     */
    inline double dot(const Vec3& a, const Vec3& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /**
     * Cross product: a x b, a vector orthogonal to both a and b
     * (right-hand rule), with magnitude equal to the area of the
     * parallelogram they span.
     * Anticommutative: cross(a, b) == -cross(b, a) (component-wise negation);
     * this is why it is never overloaded onto operator*, whose parameter
     * types (Vec3, Vec3) are already claimed by the dot product above and
     * whose symbol otherwise suggests commutativity.
     * @param a first vector.
     * @param b second vector.
     * @return a new Vec3.
     */
    inline Vec3 cross(const Vec3& a, const Vec3& b) {
        return Vec3{
            a.y*b.z - a.z*b.y,
            a.z*b.x - a.x*b.z,
            a.x*b.y - a.y*b.x
        };
    }

    /**
     * Squared Euclidean norm: ||v||^2 = v . v.
     * Avoids a square root when only the square is needed.
     * @param v the vector.
     * @return a double, always >= 0.
     */
    inline double length_squared(const Vec3& v) {
        return dot(v, v);
    }

    /**
     * Euclidean norm: ||v|| = sqrt(v . v).
     * @param v the vector.
     * @return a double, always >= 0.
     */
    inline double length(const Vec3& v) {
        return std::sqrt(length_squared(v));
    }

    /**
     * Unit-length vector in the same direction as v.
     * Precondition: ||v|| != 0 (not checked here).
     * @param v the vector to normalize.
     * @return a new Vec3 of norm 1.
     */
    inline Vec3 normalized(const Vec3& v) {
        return v * (1.0 / length(v));
    }

    /**
     * Reflects d across the plane with unit normal n: reflect(d, n) =
     * d - 2*(d.n)*n. Decomposing d = d_par + d_perp (the components parallel
     * and perpendicular to n), the reflection keeps d_par and flips d_perp,
     * so ||reflect(d, n)|| == ||d|| whatever n's length (only its direction
     * enters the formula).
     *
     * Two unrelated uses share this one formula: mirroring a ray direction
     * off a surface (d the incoming direction, pointing into the surface),
     * and the specular direction of Phong shading (d = -L, the vector
     * pointing away from the light, which is why shading::phong_light calls
     * this as reflect(L * -1.0, N) rather than duplicating the algebra).
     * @param d the vector to reflect.
     * @param n the unit normal of the reflecting plane.
     * @return a new Vec3 of the same length as d.
     */
    inline Vec3 reflect(const Vec3& d, const Vec3& n) {
        return d - n * (2.0 * dot(d, n));
    }

}  // namespace raytracer::engine