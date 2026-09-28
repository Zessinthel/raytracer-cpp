// include/raytracer/shading/lighting.hpp
#pragma once
#include <cstddef>
#include <variant>
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/scene.hpp"

namespace raytracer::shading {

    /**
     * Diffuse illumination at a surface point: the diffuse part of the
     * ILUMINACION procedure (Algorithm 2.4 of the course notes),
     *
     *     I_d = I_A + sum over lights j of  I_j * max(0, N . l_j)
     *
     * where l_j is the unit vector from the point toward light j. The cosine
     * is Lambert's law: a surface element tilted away from the light
     * intercepts proportionally less of its flux.
     *
     * The model of the notes, followed here, has no attenuation with
     * distance: a point light is as strong at 10 units as at 1 (the factor
     * a_j of the notes is 1). Only its direction matters, and that is
     * recomputed at every point for a point light, while a directional light
     * has the same direction everywhere. A light on the far side of the
     * surface, or exactly edge-on, contributes nothing. Shadows are not
     * considered yet, so a light contributes even if another object stands
     * between it and the point.
     *
     * The result can exceed 1 when the intensities add up to more than 1;
     * it is not clamped here (see engine::Color).
     *
     * @param world the scene whose ambient light and lights are used.
     * @param point the surface point.
     * @param normal the unit normal at the point, pointing to the side that
     *        is lit (outward for closed meshes).
     * @return the diffuse illumination, a number >= the ambient intensity.
     */
    inline double diffuse_light(const raytracer::scene::Scene& world,
                                const raytracer::engine::Vec3& point,
                                const raytracer::engine::Vec3& normal) {
        using namespace raytracer::engine;
        using raytracer::scene::DirectionalLight;
        using raytracer::scene::PointLight;

        double total = world.ambient();
        for (const raytracer::scene::Light& light : world.lights()) {
            Vec3 to_light{};
            double intensity = 0.0;
            if (const auto* point_light = std::get_if<PointLight>(&light)) {
                to_light = point_light->position - point;
                intensity = point_light->intensity;
            } else {
                const auto& directional = std::get<DirectionalLight>(light);
                to_light = directional.direction_to_light;
                intensity = directional.intensity;
            }

            double n_dot_l = dot(normal, to_light);      // scaled by |to_light| for now
            if (n_dot_l <= 0.0)
                continue;                                // faces away, or edge-on: no light

            // n_dot_l > 0 implies to_light is not the zero vector, so the
            // division is safe even for a point light sitting on the surface.
            total += intensity * n_dot_l / length(to_light);
        }
        return total;
    }

    /**
     * Color of a surface point under diffuse light: the material's albedo
     * scaled by the diffuse illumination. Following the notes (rho_s = rho_d)
     * one albedo tints the whole response.
     * @param world the scene.
     * @param hit the intersection to color; its normal must be a unit vector.
     * @return the color, not clamped.
     */
    inline raytracer::engine::Color diffuse_color(const raytracer::scene::Scene& world,
                                                  const raytracer::scene::SceneHit& hit) {
        const raytracer::scene::Material& material =
            world.material_at(static_cast<std::size_t>(hit.object_index));
        return material.albedo * diffuse_light(world, hit.point, hit.normal);
    }

}  // namespace raytracer::shading
