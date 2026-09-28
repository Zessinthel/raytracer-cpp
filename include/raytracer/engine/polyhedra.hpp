// include/raytracer/engine/polyhedra.hpp
#pragma once
#include <cmath>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"

namespace raytracer::engine {

    /**
     * Axis-aligned cube centered at the origin.
     * Vertices and faces are given explicitly (no regular (u,v) grid).
     * Each face is a quad split into two triangles; winding is
     * counter-clockwise seen from outside.
     * @param side edge length (default 1.0).
     * @return a TriangleMesh with 8 vertices and 12 triangles.
     */
    inline TriangleMesh cube(double side = 1.0) {
        double h = side / 2.0;
        TriangleMesh mesh;
        mesh.vertices = {
            Vec3{-h, -h, -h}, Vec3{ h, -h, -h}, Vec3{ h,  h, -h}, Vec3{-h,  h, -h},
            Vec3{-h, -h,  h}, Vec3{ h, -h,  h}, Vec3{ h,  h,  h}, Vec3{-h,  h,  h}
        };
        mesh.triangles = {
            {0, 3, 2}, {0, 2, 1},   // -z
            {4, 5, 6}, {4, 6, 7},   // +z
            {0, 1, 5}, {0, 5, 4},   // -y
            {3, 7, 6}, {3, 6, 2},   // +y
            {0, 4, 7}, {0, 7, 3},   // -x
            {1, 2, 6}, {1, 6, 5}    // +x
        };
        return mesh;
    }

    /**
     * Regular tetrahedron centered at the origin, all four vertices at
     * distance circumradius from the center. Winding is
     * counter-clockwise seen from outside.
     * @param circumradius distance from the center to each vertex (default 1.0).
     * @return a TriangleMesh with 4 vertices and 4 triangles.
     */
    inline TriangleMesh tetrahedron(double circumradius = 1.0) {
        double s = circumradius / std::sqrt(3.0);
        TriangleMesh mesh;
        mesh.vertices = {
            Vec3{ s,  s,  s}, Vec3{ s, -s, -s}, Vec3{-s,  s, -s}, Vec3{-s, -s,  s}
        };
        mesh.triangles = { {0, 1, 2}, {0, 3, 1}, {0, 2, 3}, {1, 3, 2} };
        return mesh;
    }

    /**
     * Five-pointed (or n-pointed) 3D star, a "stellated bipyramid":
     * a ring of 2*n_points vertices in the z=0 plane alternating between
     * outer_radius and inner_radius, plus one apex vertex in front
     * (z=+half_thickness) and one behind (z=-half_thickness), both on the
     * axis. One point aims along +y. Every face is a triangle joining an
     * apex to two adjacent ring vertices; winding is counter-clockwise
     * seen from outside.
     *
     * Fan triangulation of the flat outline is deliberately avoided: the
     * outline is non-convex, so a fan from a rim vertex would produce
     * triangles outside the star. Using the two axis apexes sidesteps
     * that, and gives the faceted look of a tree-top ornament.
     *
     * @param n_points number of star points (default 5, >= 3).
     * @param outer_radius distance from the axis to each tip (default 1.0).
     * @param inner_radius distance from the axis to each inner corner
     *        (default 0.381966 = regular pentagram ratio; must be < outer_radius).
     * @param half_thickness distance of each apex from the z=0 plane (default 0.3).
     * @return a closed TriangleMesh with 2*n_points+2 vertices and
     *         4*n_points triangles.
     */
    inline TriangleMesh star(int n_points = 5,
                            double outer_radius = 1.0,
                            double inner_radius = 0.381966,
                            double half_thickness = 0.3) {
        int ring = 2 * n_points;
        TriangleMesh mesh;
        mesh.vertices.reserve(ring + 2);

        for (int k = 0; k < ring; ++k) {
            double angle = M_PI / 2.0 + k * M_PI / n_points;
            double radius = (k % 2 == 0) ? outer_radius : inner_radius;
            mesh.vertices.push_back(Vec3{radius * std::cos(angle), radius * std::sin(angle), 0.0});
        }
        int front = ring;
        int back  = ring + 1;
        mesh.vertices.push_back(Vec3{0.0, 0.0,  half_thickness});
        mesh.vertices.push_back(Vec3{0.0, 0.0, -half_thickness});

        for (int k = 0; k < ring; ++k) {
            int next = (k + 1) % ring;
            mesh.triangles.push_back({front, k, next});
            mesh.triangles.push_back({back, next, k});
        }
        return mesh;
    }

}  // namespace raytracer::engine