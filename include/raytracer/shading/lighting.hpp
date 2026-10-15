// include/raytracer/shading/lighting.hpp
#pragma once
#include <cmath>
#include <cstddef>
#include <variant>
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/scene.hpp"

namespace raytracer::shading {

    /**
     * Tolerance added to a shadow ray's t_min (2.4.2): a length "del orden
     * del error de redondeo en las coordenadas de la escena", independent of
     * the distance to the light or the length of any light vector. Without
     * it, a ray leaving a surface at t_min = 0 can immediately re-intersect
     * the very triangle (or an adjacent one sharing a vertex) it started
     * from, because the hit point P = O + t*D satisfies F(P) = 0 only up to
     * floating-point rounding: the artifact is shadow acne, a dark speckle
     * over lit surfaces. The same constant will serve a reflected ray's
     * t_min in the next step, for the same reason.
     */
    inline constexpr double SHADOW_EPSILON = 1e-4;

    namespace detail {

        /**
         * The view direction and specular exponent needed for the specular
         * term (lines 9-13 of Algorithm 2.7). A null pointer to this struct,
         * passed around instead of an empty std::optional, is what lets
         * accumulate_light skip the whole specular branch for a matte
         * material or for diffuse_light without any run-time cost.
         */
        struct Specular {
            raytracer::engine::Vec3 view_direction;
            double exponent;
        };

        /**
         * Shared body of Algorithm 2.7 (ILUMINACION, with shadows): ambient
         * light plus, for each light both on the outer side of the surface
         * (n.l > 0) and not occluded, the diffuse term, and, only if
         * specular is not null, the Phong specular term. Returns their sum
         * directly, since the color is always material.albedo * (I_d + I_s)
         * here (the course model sets rho_s = rho_d).
         *
         * The occlusion test (line 7) runs only after the n.l > 0 test
         * (line 6), and before either term is added, matching the notes'
         * remark that this order avoids tracing a shadow ray toward a light
         * that could not have contributed anyway.
         */
        inline double accumulate_light(const raytracer::scene::Scene& world,
                                       const raytracer::engine::Vec3& point,
                                       const raytracer::engine::Vec3& normal,
                                       const Specular* specular) {
            using namespace raytracer::engine;
            using raytracer::scene::DirectionalLight;
            using raytracer::scene::PointLight;

            double total = world.ambient();                          // line 1
            for (const raytracer::scene::Light& light : world.lights()) {   // line 2
                Vec3 to_light{};
                double intensity = 0.0;
                bool is_point = false;
                if (const auto* point_light = std::get_if<PointLight>(&light)) {
                    to_light = point_light->position - point;         // line 3
                    intensity = point_light->intensity;
                    is_point = true;
                } else {
                    to_light = std::get<DirectionalLight>(light).direction_to_light;   // line 4
                    intensity = std::get<DirectionalLight>(light).intensity;
                }

                double dist = length(to_light);
                double n_dot_l = dot(normal, to_light);                // line 5 (unnormalized)
                if (n_dot_l <= 0.0)                                    // line 6
                    continue;                                          // inner side: no shadow ray needed

                double shadow_t_max = is_point ? dist : T_INFINITE;    // line 3/4's t_max
                Vec3 unit_l = to_light * (1.0 / dist);
                if (world.occluded(Ray{point, unit_l}, SHADOW_EPSILON, shadow_t_max))   // line 7
                    continue;                                          // in shadow: no diffuse, no specular

                total += intensity * n_dot_l / dist;                   // line 8

                if (specular) {
                    Vec3 r = reflect(unit_l * -1.0, normal);            // line 10: REFLEJAR(L, N)
                    double r_dot_v = dot(r, specular->view_direction);  // line 11 (||R|| = ||L_unit|| = 1 here)
                    if (r_dot_v > 0.0)                                  // line 12
                        total += intensity * std::pow(r_dot_v, specular->exponent);   // line 13
                }
            }
            return total;                                              // (I_d + I_s)
        }

    }  // namespace detail

    /**
     * Diffuse illumination at a surface point, with shadows: ambient light
     * plus, for each light on the outer side of the surface and not
     * occluded by another object, Lambert's cosine term (I_d of Algorithm
     * 2.7, without the specular part). See the notes (2.3.2, 2.4.2) for the
     * physical derivation, the model's a_j = 1 simplification for point
     * lights, and the shadow-ray tolerance SHADOW_EPSILON.
     *
     * The result can exceed 1 when intensities add up to more than 1; it is
     * not clamped here (see engine::Color).
     * @param world the scene whose ambient light, lights and other objects
     *        (for occlusion) are used.
     * @param point the surface point.
     * @param normal the unit normal at the point, pointing to the side that
     *        is lit (outward for closed meshes).
     * @return the diffuse illumination, a number >= the ambient intensity.
     */
    inline double diffuse_light(const raytracer::scene::Scene& world,
                                const raytracer::engine::Vec3& point,
                                const raytracer::engine::Vec3& normal) {
        return detail::accumulate_light(world, point, normal, nullptr);
    }

    /**
     * Diffuse plus specular illumination at a surface point, with shadows:
     * I_d + I_s of Algorithm 2.7. A light behind the surface (n.l <= 0) or
     * occluded by another object contributes to neither term; one on the
     * wrong side of R (R.V <= 0) contributes only its diffuse term.
     * @param world the scene.
     * @param point the surface point.
     * @param normal the unit normal at the point.
     * @param view_direction unit vector from the point toward the eye (V in
     *        the notes; the camera's rays are unit vectors, so this is
     *        -ray.direction for a primary ray).
     * @param specular_exponent the material's Phong exponent s (> 0):
     *        larger means a tighter, glossier highlight.
     * @return the total illumination, I_d + I_s (not clamped).
     */
    inline double phong_light(const raytracer::scene::Scene& world,
                              const raytracer::engine::Vec3& point,
                              const raytracer::engine::Vec3& normal,
                              const raytracer::engine::Vec3& view_direction,
                              double specular_exponent) {
        detail::Specular specular{view_direction, specular_exponent};
        return detail::accumulate_light(world, point, normal, &specular);
    }

    /**
     * Color of a surface point under diffuse light only (with shadows): the
     * material's albedo scaled by diffuse_light. Following the notes
     * (rho_s = rho_d) one albedo tints the whole response.
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

    /**
     * Color of a surface point under the full local illumination model
     * (Algorithm 2.9, line 7, with rho_s = rho_d), with shadows: the
     * material's albedo scaled by I_d + I_s. A material with no
     * specular_exponent has no lobe, so this returns exactly diffuse_color's
     * value for it, not merely a close one.
     * @param world the scene.
     * @param hit the intersection to color; its normal must be a unit vector.
     * @param view_direction unit vector from the point toward the eye.
     * @return the color, not clamped.
     */
    inline raytracer::engine::Color phong_color(const raytracer::scene::Scene& world,
                                                const raytracer::scene::SceneHit& hit,
                                                const raytracer::engine::Vec3& view_direction) {
        const raytracer::scene::Material& material =
            world.material_at(static_cast<std::size_t>(hit.object_index));
        double total = material.specular_exponent
                          ? phong_light(world, hit.point, hit.normal, view_direction, *material.specular_exponent)
                          : diffuse_light(world, hit.point, hit.normal);
        return material.albedo * total;
    }

}  // namespace raytracer::shading
