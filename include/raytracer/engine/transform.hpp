// include/raytracer/engine/transform.hpp
#pragma once
#include <utility>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"

namespace raytracer::engine {

    /**
     * Applies the affine map x -> M x + t to every vertex of a mesh.
     *
     * A map with negative determinant (a reflection, or an odd number of
     * negative scale factors) reverses orientation, so the winding of every
     * triangle is reversed as well; this keeps outward normals outward.
     * Normals are never stored, they are recomputed from the triangles, so
     * no inverse-transpose is needed here.
     *
     * @param mesh the source mesh (left untouched).
     * @param M linear part of the map (rotation, scale, shear or reflection).
     * @param t translation applied after M.
     * @return a new TriangleMesh with the transformed vertices.
     */
    inline TriangleMesh transformed(const TriangleMesh& mesh, const Mat3& M, const Vec3& t) {
        TriangleMesh result;
        result.vertices.reserve(mesh.vertices.size());
        for (const Vec3& v : mesh.vertices)
            result.vertices.push_back(M * v + t);

        result.triangles = mesh.triangles;
        if (M.determinant() < 0.0) {
            for (auto& tri : result.triangles)
                std::swap(tri[1], tri[2]);
        }
        return result;
    }

    /**
     * Translates every vertex of a mesh by t.
     * @param mesh the source mesh (left untouched).
     * @param t the displacement.
     * @return a new TriangleMesh.
     */
    inline TriangleMesh translated(const TriangleMesh& mesh, const Vec3& t) {
        return transformed(mesh, identity3(), t);
    }

}  // namespace raytracer::engine
