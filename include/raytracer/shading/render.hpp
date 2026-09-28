// include/raytracer/shading/render.hpp
#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/image.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/shading.hpp"

namespace raytracer::shading {

    /**
     * What the renderer draws for each pixel. Every mode answers the same
     * visibility question, which object is first along the ray and where; they
     * differ in what they show of the answer. None of them uses light yet.
     *
     * The modes fall in two families. The picture-like ones (normals, albedo)
     * draw the sky where nothing is hit. The data maps (distance, object_id)
     * draw black there, so that a miss is not mistaken for a value.
     */
    enum class ShadingMode {
        normals,     ///< surface normal as color, over a sky gradient
        albedo,      ///< the material's diffuse color, flat, over a sky gradient
        distance,    ///< distance t from the eye as gray, white near and black far
        object_id    ///< one flat color per object, over black
    };

    /** Everything that configures how a scene is turned into an image. */
    struct RenderSettings {
        ShadingMode mode = ShadingMode::normals;

        /** Distance shown as black in ShadingMode::distance, in scene units. */
        double distance_far = 20.0;
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
        auto hit = world.intersect(ray, 0.0, raytracer::engine::T_INFINITE);

        switch (settings.mode) {
            case ShadingMode::normals:
                return hit ? normal_to_color(*hit) : background_color(ray);
            case ShadingMode::albedo:
                return hit ? world.material_at(static_cast<std::size_t>(hit->object_index)).albedo
                           : background_color(ray);
            case ShadingMode::distance:
                return hit ? distance_to_color(hit->t, settings.distance_far) : Color{0.0, 0.0, 0.0};
            case ShadingMode::object_id:
                return hit ? object_id_color(hit->object_index) : Color{0.0, 0.0, 0.0};
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
