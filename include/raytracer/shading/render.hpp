// include/raytracer/shading/render.hpp
#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/image.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/lighting.hpp"
#include "raytracer/shading/shading.hpp"

namespace raytracer::shading {

    /**
     * What the renderer draws for each pixel. Every mode answers the same
     * visibility question, which object is first along the ray and where; they
     * differ in what they show of the answer.
     *
     * The modes fall in two families. The picture-like ones (normals, albedo,
     * lambert, phong, whitted) draw the sky where nothing is hit. The data maps
     * (distance, object_id) draw black there, so that a miss is not mistaken
     * for a value. lambert, phong and whitted use the scene's lights and
     * shadows; whitted additionally recurses into mirror reflections.
     */
    enum class ShadingMode {
        normals,     ///< surface normal as color, over a sky gradient
        albedo,      ///< the material's diffuse color, flat, over a sky gradient
        lambert,     ///< albedo times the diffuse illumination (ambient + Lambert + shadows), over a sky gradient
        phong,       ///< lambert plus the material's Phong specular highlight (also shadowed), over a sky gradient
        whitted,     ///< phong plus recursive mirror reflection (Algorithm 2.9), over a sky gradient
        distance,    ///< distance t from the eye as gray, white near and black far
        object_id    ///< one flat color per object, over black
    };

    /** Everything that configures how a scene is turned into an image. */
    struct RenderSettings {
        ShadingMode mode = ShadingMode::normals;

        /** Distance shown as black in ShadingMode::distance, in scene units. */
        double distance_far = 20.0;

        /**
         * Maximum recursion depth for ShadingMode::whitted (d_max in the
         * notes, 2.4.2/2.4.3): how many mirror bounces a ray may still take.
         * 0 disables reflection entirely (every material behaves as if its
         * reflectivity were 0), matching line 9 of Algorithm 2.9. The notes'
         * error bound is r^(d+1) times the largest possible color
         * difference; d_max = 3 keeps that under one 8-bit level for any
         * reflectivity up to 0.2, and d_max = 7 is needed up to 0.5.
         */
        int max_depth = 3;
    };

    /**
     * Distinct color for an object index. Eight colors are cycled, so objects
     * 8 apart share a color; the palette is ColorBrewer's qualitative Set1.
     * @param object_index index of the object in the scene (>= 0).
     * @return a Color.
     */
    inline raytracer::engine::Color object_id_color(int object_index) {
        using raytracer::engine::Color;
        static const std::array<Color, 8> palette = {{
            {0.894, 0.102, 0.110},   // red
            {0.216, 0.494, 0.722},   // blue
            {0.302, 0.686, 0.290},   // green
            {0.596, 0.306, 0.639},   // purple
            {1.000, 0.498, 0.000},   // orange
            {1.000, 1.000, 0.200},   // yellow
            {0.651, 0.337, 0.157},   // brown
            {0.969, 0.506, 0.749}    // pink
        }};
        return palette[static_cast<std::size_t>(object_index) % palette.size()];
    }

    /**
     * Gray level for a hit at distance t: 1 at t = 0, falling linearly to 0
     * at t = far and staying 0 beyond it.
     * @param t distance from the eye to the hit (>= 0).
     * @param far distance mapped to black (> 0).
     * @return a gray Color.
     */
    inline raytracer::engine::Color distance_to_color(double t, double far) {
        double gray = 1.0 - std::min(t / far, 1.0);
        return raytracer::engine::Color{gray, gray, gray};
    }

    /**
     * TRAZAR-RAYO (Algorithm 2.9): the recursive Whitted-style ray tracer.
     * Finds the closest hit in [t_min, t_max); if there is none, returns the
     * background. Otherwise computes the local color under the full
     * illumination model (ambient, Lambert, Phong, shadows -- Algorithm 2.7,
     * via phong_light/diffuse_light), and, if the object is reflective and
     * depth remains, blends it with the color seen in the mirror direction:
     *
     *   R = D - 2*(N.D)*N            (2.4.2's closed form; D is the
     *                                 INCIDENT direction, not V = -D, so no
     *                                 sign flip is needed here)
     *   c = sat( (1-r)*c_loc + r*trace_ray(P, R, delta, inf, depth-1) )
     *
     * c_loc itself is never clamped before this combination (2.4.2: "sin
     * recortar"); saturate() is applied once, to the value this call
     * returns, whether that is c_loc alone (depth = 0 or r = 0, line 9) or
     * the blended color (line 12) -- so every return of trace_ray is in
     * [0, 1], and a chain of reflections combines already-bounded colors at
     * each level, which is what makes the geometric error bound of 2.4.2
     * apply to a *combination* of saturated colors, not to some unbounded
     * intermediate sum.
     *
     * Termination: each recursive call receives depth-1, so a ray can
     * reflect at most max_depth times regardless of the scene -- two facing
     * mirrors with reflectivity 1 (a "hall of mirrors", which never stops
     * losing energy) still terminate, at depth 0, with whatever the
     * max_depth-th surface's local color is.
     * @param world the scene.
     * @param ray the ray to trace; its direction must be a unit vector so
     *        that t is a distance and the reflected ray's direction stays
     *        unit as well (reflection is an isometry).
     * @param t_min lower end of the search interval (inclusive).
     * @param t_max upper end of the search interval (exclusive).
     * @param depth recursion budget remaining (>= 0).
     * @return a Color in [0, 1].
     */
    inline raytracer::engine::Color trace_ray(const raytracer::scene::Scene& world,
                                              const raytracer::engine::Ray& ray,
                                              double t_min, double t_max, int depth) {
        using namespace raytracer::engine;

        auto hit = world.intersect(ray, t_min, t_max);           // line 1
        if (!hit)
            return background_color(ray);                        // line 2

        const raytracer::scene::Material& material =
            world.material_at(static_cast<std::size_t>(hit->object_index));
        Vec3 view = ray.direction * -1.0;                         // line 5: V = -D
        double total = material.specular_exponent                 // line 6
                          ? phong_light(world, hit->point, hit->normal, view, *material.specular_exponent)
                          : diffuse_light(world, hit->point, hit->normal);
        Color local = material.albedo * total;                    // line 7, not clamped yet

        double r = material.reflectivity;                          // line 8
        if (depth == 0 || r == 0.0)                                 // line 9
            return saturate(local);

        Vec3 reflected_direction = reflect(ray.direction, hit->normal);   // line 10, closed form
        Color reflected = trace_ray(world, Ray{hit->point, reflected_direction},
                                    SHADOW_EPSILON, T_INFINITE, depth - 1);   // line 11
        return saturate(local * (1.0 - r) + reflected * r);         // line 12
    }

    /**
     * Colors a single ray according to the render settings.
     * @param ray the ray to shade; its direction should be a unit vector so
     *        that t is a distance.
     * @param world the scene to test against.
     * @param settings which mode to draw and its parameters.
     * @return a Color.
     */
    inline raytracer::engine::Color shade(const raytracer::engine::Ray& ray,
                                          const raytracer::scene::Scene& world,
                                          const RenderSettings& settings) {
        using raytracer::engine::Color;
        using raytracer::engine::T_INFINITE;

        switch (settings.mode) {
            case ShadingMode::normals: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                return hit ? normal_to_color(*hit) : background_color(ray);
            }
            case ShadingMode::albedo: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                return hit ? world.material_at(static_cast<std::size_t>(hit->object_index)).albedo
                           : background_color(ray);
            }
            case ShadingMode::lambert: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                return hit ? diffuse_color(world, *hit) : background_color(ray);
            }
            case ShadingMode::phong: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                // ray.direction is a unit vector (Camera's rays are), so -ray.direction
                // is already the unit V of the notes; no renormalization needed.
                return hit ? phong_color(world, *hit, ray.direction * -1.0) : background_color(ray);
            }
            case ShadingMode::whitted:
                return trace_ray(world, ray, 0.0, T_INFINITE, settings.max_depth);
            case ShadingMode::distance: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                return hit ? distance_to_color(hit->t, settings.distance_far) : Color{0.0, 0.0, 0.0};
            }
            case ShadingMode::object_id: {
                auto hit = world.intersect(ray, 0.0, T_INFINITE);
                return hit ? object_id_color(hit->object_index) : Color{0.0, 0.0, 0.0};
            }
        }
        return Color{0.0, 0.0, 0.0};   // not reached: every mode returns above
    }

    /**
     * Renders the scene as seen by the camera. This is the loop over pixels
     * of the course notes: one primary ray per pixel, each independent of
     * the others. The camera indexes pixels with j = 0 at the bottom of the
     * viewport while Image stores row 0 at the top, so the flip happens here,
     * once.
     * @param camera defines the viewport and the resolution.
     * @param world the scene to render.
     * @param settings which mode to draw and its parameters.
     * @return the rendered Image, camera.width() by camera.height().
     */
    inline raytracer::engine::Image render(const raytracer::scene::Camera& camera,
                                           const raytracer::scene::Scene& world,
                                           const RenderSettings& settings) {
        raytracer::engine::Image image(camera.width(), camera.height());
        for (int j = 0; j < camera.height(); ++j)
            for (int i = 0; i < camera.width(); ++i)
                image.at(i, camera.height() - 1 - j) = shade(camera.ray_for_pixel(i, j), world, settings);
        return image;
    }

}  // namespace raytracer::shading
