// include/raytracer/physics/triangle.hpp
#pragma once
#include <optional>
#include <cmath>
#include "raytracer/engine/vec3.hpp"

namespace raytracer::physics {

    /**
     * Result of a successful ray-triangle intersection: the ray
     * parameter t, and the barycentric coordinates (u, v) of the hit
     * point within the triangle (the third barycentric weight is
     * 1 - u - v).
     */
    struct TriangleHit {
        double t, u, v;
    };

    /**
     * Moller-Trumbore ray-triangle intersection: closed-form algebraic
     * solution via barycentric coordinates and Cramer's rule
     * (det[a b c] = a . (b x c)), no iteration.
     *
     * Only hits with t in the half-open interval [t_min, t_max) count. The
     * interval is what lets callers ask different questions with the same
     * routine: t_min > 0 keeps a secondary ray from re-hitting the surface
     * it starts on, and t_max lets a caller discard anything farther than the
     * best hit found so far. When D is a unit vector, t is a distance and
     * the interval is a range of lengths.
     * @param O ray origin.
     * @param D ray direction (the algebra does not require it to be
     *          normalized; the meaning of the interval does).
     * @param v0, v1, v2 triangle vertices.
     * @param t_min lower end of the accepted interval (inclusive).
     * @param t_max upper end of the accepted interval (exclusive); use
     *              engine::T_INFINITE for no upper bound.
     * @param eps tolerance for the degenerate case (ray parallel to
     *            the triangle's plane, det ~= 0).
     * @return a TriangleHit if the ray hits the triangle at some t in
     *         [t_min, t_max) with valid barycentric coordinates;
     *         std::nullopt otherwise.
     */
    inline std::optional<TriangleHit> intersect_triangle(
        const raytracer::engine::Vec3& O,
        const raytracer::engine::Vec3& D,
        const raytracer::engine::Vec3& v0,
        const raytracer::engine::Vec3& v1,
        const raytracer::engine::Vec3& v2,
        double t_min,
        double t_max,
        double eps = 1e-9)
    {
        using namespace raytracer::engine;

        Vec3 e1 = v1 - v0;
        Vec3 e2 = v2 - v0;
        Vec3 pvec = cross(D, e2);
        double det = dot(e1, pvec);

        if (std::abs(det) < eps)
            return std::nullopt;

        double inv_det = 1.0 / det;
        Vec3 tvec = O - v0;
        double u = dot(tvec, pvec) * inv_det;
        if (u < 0.0 || u > 1.0)
            return std::nullopt;

        Vec3 qvec = cross(tvec, e1);
        double v = dot(D, qvec) * inv_det;
        if (v < 0.0 || u + v > 1.0)
            return std::nullopt;

        double t = dot(e2, qvec) * inv_det;
        // Written as a negated conjunction so that a NaN t is rejected too.
        if (!(t >= t_min && t < t_max))
            return std::nullopt;

        return TriangleHit{t, u, v};
    }

}  // namespace raytracer::physics