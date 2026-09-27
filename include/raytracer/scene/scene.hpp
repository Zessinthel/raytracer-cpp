// include/raytracer/scene/scene.hpp
#pragma once
#include <vector>
#include <optional>
#include <limits>
#include "raytracer/engine/ray.hpp"
#include "raytracer/physics/mesh.hpp"

namespace raytracer::scene {

/**
 * Result of a successful ray-scene intersection: everything
 * shading needs to color the hit point, plus which object was hit.
 */
struct SceneHit {
    double t;
    raytracer::engine::Vec3 point;
    raytracer::engine::Vec3 normal;
    int object_index;
};

/**
 * A collection of intersectable meshes. Finds the closest valid
 * hit across all of them for a given ray, so the caller never has
 * to know how many objects exist or iterate them by hand.
 */
class Scene {
public:
    /**
     * Adds a mesh to the scene, taking ownership of it (moved in).
     * @param mesh the mesh to add.
     * @return the index assigned to this object, usable later to
     *         look it up via object_at or to identify a SceneHit.
     */
    int add(raytracer::physics::Mesh mesh) {
        objects_.push_back(std::move(mesh));
        return static_cast<int>(objects_.size() - 1);
    }

    /**
     * Finds the closest valid intersection (smallest t >= 0) between
     * the ray and any object in the scene.
     * @param ray the ray to test.
     * @return a SceneHit if some object is hit; std::nullopt otherwise.
     */
    std::optional<SceneHit> intersect(const raytracer::engine::Ray& ray) const {
        std::optional<SceneHit> closest;

        for (std::size_t i = 0; i < objects_.size(); ++i) {
            auto hit = objects_[i].intersect(ray);
            if (!hit)
                continue;
            if (!closest || hit->t < closest->t) {
                closest = SceneHit{
                    hit->t,
                    ray.at(hit->t),
                    hit->normal,
                    static_cast<int>(i)
                };
            }
        }

        return closest;
    }

    /** @return number of objects currently in the scene. */
    std::size_t object_count() const { return objects_.size(); }

    /** @return read-only access to object i, for inspection. */
    const raytracer::physics::Mesh& object_at(std::size_t i) const {
        return objects_[i];
    }

private:
    std::vector<raytracer::physics::Mesh> objects_;
};

}  // namespace raytracer::scene