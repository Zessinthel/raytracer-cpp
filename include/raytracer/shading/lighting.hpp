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

    namespace detail {

        /**
         * The view direction and specular exponent needed for the specular
         * term (lines 8-12 of Algorithm 2.4). A null pointer to this struct,
         * passed around instead of an empty std::optional, is what lets
         * accumulate_light skip the whole specular branch for a matte
         * material or for diffuse_light without any run-time cost.
         */
        struct Specular {
            raytracer::engine::Vec3 view_direction;
            double exponent;
        };

        /**
         * Shared body of Algorithm 2.4 (ILUMINACION): ambient light plus, for
         * each light on the outer side of the surface (n.l > 0), the diffuse
         * term, and, only if specular is not null, the Phong specular term.
         * Returns their sum directly, since the color is always
         * material.albedo * (I_d + I_s) here (the course model sets
         * rho_s = rho_d, so nothing is lost by adding them before the
         * multiplication instead of after, as the notes do).
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
                if (const auto* point_light = std::get_if<PointLight>(&light)) {
                    to_light = point_light->position - point;         // line 3 (a_j = 1 in this model)
                    intensity = point_light->intensity;
                } else {
                    to_light = std::get<DirectionalLight>(light).direction_to_light;   // line 4
                    intensity = std::get<DirectionalLight>(light).intensity;
                }

                double dist = length(to_light);
                double n_dot_l = dot(normal, to_light);                // line 5 (unnormalized)
                if (n_dot_l <= 0.0)                                    // line 6
                    continue;                                          // inner side: no diffuse, no specular

                total += intensity * n_dot_l / dist;                   // line 7

                if (specular) {
                    Vec3 unit_l = to_light * (1.0 / dist);
                    Vec3 r = reflect(unit_l * -1.0, normal);            // line 9: 2(N.L)N - L, via engine::reflect
                    double r_dot_v = dot(r, specular->view_direction);  // line 10 (||R|| = ||L_unit|| = 1 here)
                    if (r_dot_v > 0.0)                                  // line 11
                        total += intensity * std::pow(r_dot_v, specular->exponent);   // line 12
                }
            }
            return total;                                              // (I_d + I_s), see the note above
        }

    }  // namespace detail

    /**
     * Diffuse illumination at a surface point: ambient light plus, for each
     * light on the outer side of the surface, Lambert's cosine term (the
     * I_d of Algorithm 2.4, without the specular part). See the notes
     * (2.3.2) for the physical derivation and the model's a_j = 1
     * simplification for point lights.
     *
     * The result can exceed 1 when intensities add up to more than 1; it is
     * not clamped here (see engine::Color).
     * @param world the scene whose ambient light and lights are used.
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
     * Diffuse plus specular illumination at a surface point: I_d + I_s of
     * Algorithm 2.4, with the Phong specular lobe cos^s(alpha) around the
     * mirror direction R of each light. alpha is the angle between R and
     * view_direction; a light behind the surface (n.l <= 0) contributes to
     * neither term, and one on the wrong side of R (R.V <= 0) contributes
     * only its diffuse term, per the notes' remark that this second
     * restriction is not redundant with the first.
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
     * Color of a surface point under diffuse light only: the material's
     * albedo scaled by diffuse_light. Following the notes (rho_s = rho_d)
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

    /**
     * Color of a surface point under the full local illumination model
     * (Algorithm 2.5, line 7, with rho_s = rho_d): the material's albedo
     * scaled by I_d + I_s. A material with no specular_exponent has no
     * lobe (H7 in the notes: "los materiales mate carecen de lobulo
     * especular"), so this returns exactly diffuse_color's value for it,
     * not merely a close one, which is why the mode is safe to use as the
     * one shading mode for a scene mixing matte and glossy materials.
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
