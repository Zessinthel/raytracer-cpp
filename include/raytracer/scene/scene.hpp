// include/raytracer/scene/scene.hpp
#pragma once
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <cmath>
#include <utility>
#include <vector>
#include "raytracer/engine/ray.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/material.hpp"

namespace raytracer::scene {

    /**
     * Result of a successful ray-scene intersection: everything
     * shading needs to color the hit point, plus which object was hit.
     * The object's material is looked up with Scene::material_at(object_index).
     */
    struct SceneHit {
        double t;
        raytracer::engine::Vec3 point;
        raytracer::engine::Vec3 normal;
        int object_index;
    };

    /**
     * Everything that is in the world being rendered: meshes with their
     * materials, the lights, and the ambient light. Finds the closest hit
     * across all the meshes for a given ray, so the caller never has to know
     * how many objects exist or iterate them by hand.
     *
     * Everything enters through add, add_light and set_ambient, which reject
     * invalid data, so a Scene never holds a material or a light outside the
     * ranges the illumination model assumes.
     */
    class Scene {
    public:
        /**
         * Adds a mesh with its material, taking ownership of the mesh (moved in).
         * @param mesh the mesh to add.
         * @param material how the surface responds to light; the default is a
         *        matte, non-reflective mid gray, so a scene that is only
         *        drawn with geometric modes needs no materials.
         * @return the index assigned to this object, usable later to look it
         *         up via object_at and material_at or to identify a SceneHit.
         * @throws std::invalid_argument if the material is not valid (see
         *         is_valid); the scene is left unchanged.
         */
        int add(raytracer::physics::Mesh mesh, Material material = Material{}) {
            if (!is_valid(material))
                throw std::invalid_argument(
                    "Scene::add: invalid material (albedo and reflectivity must lie in [0, 1], "
                    "the specular exponent must be finite and not negative)");
            objects_.push_back(Object{std::move(mesh), material});
            return static_cast<int>(objects_.size() - 1);
        }

        /**
         * Adds a light.
         * @param light a point or directional light.
         * @return the index of the light in lights().
         * @throws std::invalid_argument if the light is not valid (see
         *         is_valid); the scene is left unchanged.
         */
        int add_light(Light light) {
            if (!is_valid(light))
                throw std::invalid_argument(
                    "Scene::add_light: invalid light (finite non-negative intensity, finite position, "
                    "non-zero finite direction)");
            lights_.push_back(light);
            return static_cast<int>(lights_.size() - 1);
        }

        /**
         * Sets the ambient light: a uniform illumination that reaches every
         * surface point regardless of position or orientation. It is 0 until set.
         * @param intensity a finite, non-negative number.
         * @throws std::invalid_argument if the intensity is negative, NaN or
         *         infinite; the previous value is kept.
         */
        void set_ambient(double intensity) {
            if (!(std::isfinite(intensity) && intensity >= 0.0))
                throw std::invalid_argument("Scene::set_ambient: intensity must be finite and not negative");
            ambient_ = intensity;
        }

        /** @return the ambient light intensity. */
        double ambient() const { return ambient_; }

        /** @return the point and directional lights, in the order they were added. */
        const std::vector<Light>& lights() const { return lights_; }

        /**
         * Finds the closest intersection with t in [t_min, t_max) between
         * the ray and any object in the scene. The interval narrows as objects
         * are visited: each object is asked only for hits closer than the best
         * one found so far (the PRIMER-IMPACTO scheme of the course notes).
         * @param ray the ray to test; with a unit direction, t is a distance.
         * @param t_min lower end of the accepted interval (inclusive).
         * @param t_max upper end of the accepted interval (exclusive); use
         *              engine::T_INFINITE for no upper bound.
         * @return a SceneHit if some object is hit within the interval;
         *         std::nullopt otherwise.
         */
        std::optional<SceneHit> intersect(const raytracer::engine::Ray& ray,
                                          double t_min, double t_max) const {
            std::optional<SceneHit> closest;
            double limit = t_max;

            for (std::size_t i = 0; i < objects_.size(); ++i) {
                auto hit = objects_[i].mesh.intersect(ray, t_min, limit);
                if (!hit)
                    continue;
                closest = SceneHit{
                    hit->t,
                    ray.at(hit->t),
                    hit->normal,
                    static_cast<int>(i)
                };
                limit = hit->t;
            }

            return closest;
        }

        /**
         * Tells whether any object in the scene is hit by the ray with t in
         * [t_min, t_max), stopping at the first one found (OCLUIDO in the
         * course notes). Use it for shadow rays, where only the existence of
         * an obstacle matters.
         * @param ray the ray to test; with a unit direction, t is a distance.
         * @param t_min lower end of the accepted interval (inclusive).
         * @param t_max upper end of the accepted interval (exclusive); use
         *              engine::T_INFINITE for no upper bound.
         * @return true if some object is hit within the interval.
         */
        bool occluded(const raytracer::engine::Ray& ray, double t_min, double t_max) const {
            for (const auto& object : objects_) {
                if (object.mesh.occluded(ray, t_min, t_max))
                    return true;
            }
            return false;
        }

        /** @return number of objects currently in the scene. */
        std::size_t object_count() const { return objects_.size(); }

        /** @return read-only access to the mesh of object i, for inspection. */
        const raytracer::physics::Mesh& object_at(std::size_t i) const {
            return objects_[i].mesh;
        }

        /** @return the material of object i. */
        const Material& material_at(std::size_t i) const {
            return objects_[i].material;
        }

    private:
        struct Object {
            raytracer::physics::Mesh mesh;
            Material material;
        };

        std::vector<Object> objects_;
        std::vector<Light> lights_;
        double ambient_ = 0.0;
    };

}  // namespace raytracer::scene
