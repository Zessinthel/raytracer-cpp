// include/raytracer/physics/mesh.hpp
#pragma once
#include <vector>
#include <array>
#include <optional>
#include <cstddef>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/physics/triangle.hpp"

namespace raytracer::physics {

    /**
     * Result of a successful ray-mesh intersection: the ray parameter t,
     * the flat (per-face) normal at the hit triangle, and the index of
     * the triangle hit within the mesh's internal triangle list.
     */
    struct MeshHit {
        double t;
        raytracer::engine::Vec3 normal;
        int triangle_index;
    };

    /**
     * A triangulated surface ready for ray intersection: a vertex
     * buffer plus a triangle index buffer, with degenerate
     * (near-zero-area) triangles removed at construction time.
     *
     * Construct via from_triangle_mesh, never directly — this keeps
     * the "no degenerate triangles" invariant enforced at the single
     * point where triangles enter the mesh.
     */
    class Mesh {
    public:
        /**
         * Builds a Mesh from raw geometric data (e.g. the output of
         * engine::sphere, engine::cylinder, etc.), discarding any
         * triangle whose area falls below area_eps.
         *
         * Where a ring of the source parametrization collapses to a point
         * (the poles of engine::sphere, the apex of engine::cone), each quad
         * touching that ring splits into one zero-area triangle, removed
         * here, and one valid triangle that reaches the collapsed point. The
         * surface therefore stays covered and no vertex is left orphaned.
         *
         * NOTE: a ray passing exactly through a vertex or an edge shared by
         * several triangles (for instance a ray along a sphere's axis) may
         * slip through under floating-point rounding, because the
         * barycentric tests in intersect_triangle are not watertight. Rays
         * not aligned with mesh vertices are unaffected.
         *
         * @param source vertex buffer + raw triangle list.
         * @param area_eps minimum accepted triangle area (parallelogram
         *        area = |cross(e1,e2)|, i.e. twice the triangle area;
         *        default 1e-12 rejects only true degeneracies, not
         *        merely small triangles).
         * @return a Mesh containing only non-degenerate triangles.
         */
        static Mesh from_triangle_mesh(const raytracer::engine::TriangleMesh& source,
                                    double area_eps = 1e-12) {
            using namespace raytracer::engine;

            Mesh mesh;
            mesh.vertices_ = source.vertices;

            for (const auto& tri : source.triangles) {
                const Vec3& v0 = source.vertices[tri[0]];
                const Vec3& v1 = source.vertices[tri[1]];
                const Vec3& v2 = source.vertices[tri[2]];
                double parallelogram_area = length(cross(v1 - v0, v2 - v0));
                if (parallelogram_area < area_eps)
                    continue;  // degenerate: skip, do not add
                mesh.triangles_.push_back(tri);
            }

            return mesh;
        }

        /**
         * Finds the closest intersection with t in [t_min, t_max) between
         * the ray and any triangle of this mesh, via brute-force iteration
         * over every triangle (Moller-Trumbore per triangle). Each hit found
         * narrows the upper end of the interval for the triangles still to
         * be tested, so a triangle farther than the best hit so far is
         * rejected without further work and no hit can lose to an earlier one.
         * The returned normal is the geometric normal given by the triangle
         * winding (outward for closed engine primitives); it is never
         * flipped toward the ray, so it still tells inside from outside.
         * @param ray the ray to test; with a unit direction, t is a distance.
         * @param t_min lower end of the accepted interval (inclusive).
         * @param t_max upper end of the accepted interval (exclusive); use
         *              engine::T_INFINITE for no upper bound.
         * @return a MeshHit if some triangle is hit within the interval;
         *         std::nullopt otherwise.
         */
        std::optional<MeshHit> intersect(const raytracer::engine::Ray& ray,
                                         double t_min, double t_max) const {
            using namespace raytracer::engine;

            std::optional<MeshHit> closest;
            double limit = t_max;

            for (std::size_t i = 0; i < triangles_.size(); ++i) {
                const auto& tri = triangles_[i];
                const Vec3& v0 = vertices_[tri[0]];
                const Vec3& v1 = vertices_[tri[1]];
                const Vec3& v2 = vertices_[tri[2]];

                auto hit = intersect_triangle(ray.origin, ray.direction, v0, v1, v2, t_min, limit);
                if (!hit)
                    continue;

                // The interval already excludes anything not closer than the
                // best hit so far, so this one is the new closest.
                Vec3 face_normal = normalized(cross(v1 - v0, v2 - v0));
                closest = MeshHit{hit->t, face_normal, static_cast<int>(i)};
                limit = hit->t;
            }

            return closest;
        }

        /** @return number of vertices in this mesh. */
        std::size_t vertex_count() const { return vertices_.size(); }

        /** @return number of (non-degenerate) triangles in this mesh. */
        std::size_t triangle_count() const { return triangles_.size(); }

    private:
        std::vector<raytracer::engine::Vec3> vertices_;
        std::vector<std::array<int, 3>> triangles_;
    };

}  // namespace raytracer::physics