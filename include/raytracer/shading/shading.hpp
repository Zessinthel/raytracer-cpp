// include/raytracer/shading/shading.hpp
#pragma once
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/scene/scene.hpp"

namespace raytracer::shading {

    /**
     * Background color for rays that hit nothing: a vertical gradient
     * from white (down, -z) to soft blue (up, +z), based on the ray's own
     * direction rather than any scene data — a simple visual cue that
     * "nothing was hit" without being flat black.
     * @param ray the ray that missed every object in the scene.
     * @return a Color.
     */
    inline raytracer::engine::Color background_color(const raytracer::engine::Ray& ray) {
        using raytracer::engine::normalized;
        raytracer::engine::Vec3 unit_direction = normalized(ray.direction);
        double t = 0.5 * (unit_direction.z + 1.0);
        return raytracer::engine::Color{
            (1.0 - t) * 1.0 + t * 0.5,
            (1.0 - t) * 1.0 + t * 0.7,
            (1.0 - t) * 1.0 + t * 1.0
        };
    }

    /**
     * Maps a surface normal to a color by remapping each component
     * from [-1,1] to [0,1]: color = (normal + 1) / 2. Requires no
     * light source or reflection model — useful as a first visual
     * check that geometry and normals are correct before any real
     * illumination is implemented.
     * @param hit a scene intersection.
     * @return a Color derived purely from hit.normal.
     */
    inline raytracer::engine::Color normal_to_color(const raytracer::scene::SceneHit& hit) {
        return raytracer::engine::Color{
            0.5 * (hit.normal.x + 1.0),
            0.5 * (hit.normal.y + 1.0),
            0.5 * (hit.normal.z + 1.0)
        };
    }

}  // namespace raytracer::shading
