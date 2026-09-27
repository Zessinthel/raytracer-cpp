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
     * KNOWN LIMITATION (pending fix): discarding degenerate
     * triangles leaves a real hole in the surface wherever a ring
     * of the source parametrization collapses to a point (e.g. the
     * two poles of engine::sphere) — the vertices of that ring
     * survive in the buffer (orphaned, referenced by zero
     * triangles), but no triangle covers that region anymore. A ray
     * traveling exactly along the parametrization's axis of
     * symmetry passes clean through the hole regardless of grid
     * resolution, since the hole is always centered on that axis.
     * TODO: cap each pole with a fan of non-degenerate triangles,
     * built from the first non-collapsed ring plus a single new
     * apex vertex at the exact pole point — not from the collapsed
     * ring itself. This must be implemented in engine::sphere (and
     * any future parametrization with a collapsing ring), not here;
     * from_triangle_mesh should keep filtering purely by area.
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
     * Finds the closest valid intersection (smallest t >= 0) between
     * the ray and any triangle of this mesh, via brute-force
     * iteration over every triangle (Moller-Trumbore per triangle).
     * @param ray the ray to test.
     * @return a MeshHit if some triangle is hit; std::nullopt otherwise.
     */
    std::optional<MeshHit> intersect(const raytracer::engine::Ray& ray) const {
        using namespace raytracer::engine;

        std::optional<MeshHit> closest;

        for (std::size_t i = 0; i < triangles_.size(); ++i) {
            const auto& tri = triangles_[i];
            const Vec3& v0 = vertices_[tri[0]];
            const Vec3& v1 = vertices_[tri[1]];
            const Vec3& v2 = vertices_[tri[2]];

            auto hit = intersect_triangle(ray.origin, ray.direction, v0, v1, v2);
            if (!hit)
                continue;

            if (!closest || hit->t < closest->t) {
                Vec3 face_normal = normalized(cross(v1 - v0, v2 - v0));
                closest = MeshHit{hit->t, face_normal, static_cast<int>(i)};
            }
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