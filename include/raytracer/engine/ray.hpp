// include/raytracer/engine/ray.hpp
#pragma once
#include <limits>
#include "raytracer/engine/vec3.hpp"

namespace raytracer::engine {

    /**
     * Upper end of an unbounded search interval [t_min, T_INFINITE): the
     * ray is followed as far as it goes. Used wherever no object is known
     * yet to bound the search (primary rays, reflected rays).
     */
    inline constexpr double T_INFINITE = std::numeric_limits<double>::infinity();

    /**
     * A ray: an origin point plus a direction, parametrized as
     * P(t) = origin + t * direction. Pure geometry — no notion of
     * what it might intersect. When direction is a unit vector, t is the
     * distance from the origin, and search intervals [t_min, t_max) are
     * lengths measured along the ray.
     */
    struct Ray {
        Vec3 origin;
        Vec3 direction;

        /**
         * Point along the ray at parameter t.
         * @param t the ray parameter.
         * @return origin + t * direction.
         */
        Vec3 at(double t) const {
            return origin + direction * t;
        }
    };

}  // namespace raytracer::engine