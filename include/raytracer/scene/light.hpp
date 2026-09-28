// include/raytracer/scene/light.hpp
#pragma once
#include <cmath>
#include <variant>
#include "raytracer/engine/vec3.hpp"

namespace raytracer::scene {

    /**
     * A light at a point of the scene that radiates in all directions. The
     * direction from a surface point to the light changes from point to point.
     */
    struct PointLight {
        raytracer::engine::Vec3 position;
        double intensity;
    };

    /**
     * A light so far away that its rays are parallel, like the sun. It is
     * described by the direction from the scene TOWARD the light; the vector
     * does not have to be a unit vector, it is normalized where it is used.
     */
    struct DirectionalLight {
        raytracer::engine::Vec3 direction_to_light;
        double intensity;
    };

    /**
     * Either kind of light. Ambient light is not a Light: it has no position
     * or direction and is a single number of the scene (Scene::ambient).
     */
    using Light = std::variant<PointLight, DirectionalLight>;

    namespace detail {

        inline bool is_finite(const raytracer::engine::Vec3& v) {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        inline bool is_valid_intensity(double intensity) {
            return std::isfinite(intensity) && intensity >= 0.0;
        }

    }  // namespace detail

    /**
     * Checks that a light can be used: a finite, non-negative intensity, a
     * finite position for a point light, and a finite, non-zero direction for
     * a directional light (a zero vector points nowhere).
     * @param light the light to check.
     * @return true if the light is valid.
     */
    inline bool is_valid(const Light& light) {
        if (const auto* point = std::get_if<PointLight>(&light))
            return detail::is_finite(point->position) && detail::is_valid_intensity(point->intensity);

        const auto& directional = std::get<DirectionalLight>(light);
        return detail::is_finite(directional.direction_to_light) &&
               raytracer::engine::length_squared(directional.direction_to_light) > 0.0 &&
               detail::is_valid_intensity(directional.intensity);
    }

}  // namespace raytracer::scene
