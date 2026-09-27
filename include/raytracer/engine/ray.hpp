// include/raytracer/engine/ray.hpp
#pragma once
#include "raytracer/engine/vec3.hpp"

namespace raytracer::engine {

/**
 * A ray: an origin point plus a direction, parametrized as
 * P(t) = origin + t * direction. Pure geometry — no notion of
 * what it might intersect.
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