// include/raytracer/shading/shading.hpp
#pragma once
#include "raytracer/engine/vec3.hpp"
#include "raytracer/scene/scene.hpp"

namespace raytracer::shading {

    /**
     * An RGB color, each channel in [0, 1]. Kept distinct from
     * engine::Vec3 despite the identical layout: a color is not a
     * point or direction in R^3, and giving it its own type prevents
     * accidentally passing a color where geometry is expected, or
     * vice versa.
     */
    struct Color {
        double r, g, b;
    };

    /**
     * Background color for rays that hit nothing: a vertical gradient
     * from white (bottom) to soft blue (top), based on the ray's own
     * direction rather than any scene data — a simple visual cue that
     * "nothing was hit" without being flat black.
     * @param ray the ray that missed every object in the scene.
     * @return a Color.
     */
    inline Color background_color(const raytracer::engine::Ray& ray) {
        using raytracer::engine::normalized;
        raytracer::engine::Vec3 unit_direction = normalized(ray.direction);
        double t = 0.5 * (unit_direction.y + 1.0);
        return Color{
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
    inline Color normal_to_color(const raytracer::scene::SceneHit& hit) {
        return Color{
            0.5 * (hit.normal.x + 1.0),
            0.5 * (hit.normal.y + 1.0),
            0.5 * (hit.normal.z + 1.0)
        };
    }

    /**
     * Shades a single ray against a scene: normal-based color if it
     * hits something, background gradient otherwise.
     * @param ray the ray to shade.
     * @param world the scene to test against.
     * @return a Color.
     */
    inline Color shade(const raytracer::engine::Ray& ray, const raytracer::scene::Scene& world) {
        auto hit = world.intersect(ray);
        if (hit)
            return normal_to_color(*hit);
        return background_color(ray);
    }

}  // namespace raytracer::shading