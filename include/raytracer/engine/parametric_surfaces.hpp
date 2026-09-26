// include/raytracer/engine/parametric_surfaces.hpp
#pragma once
#include <cmath>
#include <vector>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/coordinates.hpp"
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/topology.hpp"

namespace raytracer::engine {

/**
 * A triangulated surface: a flat vertex buffer plus a triangle
 * index buffer. This is pure geometric data — no notion of
 * material, transform, or intersection; see physics::Mesh for
 * the primitive that wraps this with those responsibilities.
 */
struct TriangleMesh {
    std::vector<Vec3> vertices;
    TriangleList triangles;
};

/**
 * Rectangular plane in z=0, centered at the origin.
 * Parametrization: x(i) in [-width/2, width/2], y(j) in
 * [-height/2, height/2], z = 0.
 * @param nx, ny grid resolution (>= 2 each).
 * @param width, height extents (default 1.0 each).
 * @return a TriangleMesh with nx*ny vertices and
 *         2*(nx-1)*(ny-1) triangles.
 */
inline TriangleMesh plane(int nx, int ny, double width = 1.0, double height = 1.0) {
    auto xs = linspace(-width / 2.0, width / 2.0, nx);
    auto ys = linspace(-height / 2.0, height / 2.0, ny);

    TriangleMesh mesh;
    mesh.vertices.reserve(nx * ny);
    for (double x : xs)
        for (double y : ys)
            mesh.vertices.push_back(Vec3{x, y, 0.0});

    mesh.triangles = grid_triangles_open(nx, ny);
    return mesh;
}

/**
 * Sphere of radius r, centered at the origin. Parametrization:
 *   x = r*sin(theta)*cos(phi), y = r*sin(theta)*sin(phi), z = r*cos(theta)
 * with theta (colatitude) in [0, pi], phi (azimuth) in [0, 2*pi).
 * Topology: sphere_triangles — closed_phi body plus fan-triangulated
 * caps at the poles, where the grid degenerates to a point.
 * @param n_theta points in colatitude, including poles (>= 3).
 * @param n_phi points in azimuth (>= 3).
 * @param radius sphere radius (default 1.0).
 * @return a TriangleMesh with n_theta*n_phi vertices.
 */
inline TriangleMesh sphere(int n_theta, int n_phi, double radius = 1.0) {
    auto thetas = linspace(0.0, M_PI, n_theta);
    auto phis   = linspace(0.0, 2.0 * M_PI, n_phi, false);

    TriangleMesh mesh;
    mesh.vertices.reserve(n_theta * n_phi);
    for (double theta : thetas)
        for (double phi : phis)
            mesh.vertices.push_back(spherical_to_cartesian(radius, theta, phi));

    mesh.triangles = sphere_triangles(n_theta, n_phi);
    return mesh;
}

/**
 * Right circular cylinder, centered at the origin, axis along z.
 * Parametrization: x = r*cos(phi), y = r*sin(phi), z in
 * [-height/2, height/2], phi in [0, 2*pi).
 * Topology: grid_triangles_closed_phi for the lateral body, plus
 * fan-triangulated circular caps at both ends.
 * @param n_z axial points, including both ends (>= 2).
 * @param n_theta angular points (>= 3).
 * @param radius cylinder radius (default 1.0).
 * @param height total height (default 2.0).
 * @return a TriangleMesh with n_z*n_theta vertices.
 */
inline TriangleMesh cylinder(int n_z, int n_theta, double radius = 1.0, double height = 2.0) {
    auto zs   = linspace(-height / 2.0, height / 2.0, n_z);
    auto phis = linspace(0.0, 2.0 * M_PI, n_theta, false);

    TriangleMesh mesh;
    mesh.vertices.reserve(n_z * n_theta);
    for (double z : zs)
        for (double phi : phis)
            mesh.vertices.push_back(cylindrical_to_cartesian(radius, phi, z));

    mesh.triangles = grid_triangles_closed_phi(n_z, n_theta);

    Polygon bottom, top;
    for (int j = 0; j < n_theta; ++j) {
        bottom.push_back(grid_index(0, j, n_theta));
        top.push_back(grid_index(n_z - 1, j, n_theta));
    }
    for (const auto& t : triangulate_polygon(bottom)) mesh.triangles.push_back(t);
    for (const auto& t : triangulate_polygon(top))    mesh.triangles.push_back(t);

    return mesh;
}

/**
 * Right circular cone, axis along z, apex at z=+height/2, base at
 * z=-height/2. Radius decreases linearly from radius_base at the
 * base to 0 at the apex.
 * Topology: grid_triangles_closed_phi for the lateral body, plus a
 * fan-triangulated circular cap at the base only (the apex ring
 * already degenerates to a point, needing no separate cap).
 * @param n_z axial points, including base and apex (>= 2).
 * @param n_theta angular points (>= 3).
 * @param radius_base base radius (default 1.0).
 * @param height total height (default 2.0).
 * @return a TriangleMesh with n_z*n_theta vertices.
 */
inline TriangleMesh cone(int n_z, int n_theta, double radius_base = 1.0, double height = 2.0) {
    auto zs   = linspace(-height / 2.0, height / 2.0, n_z);
    auto phis = linspace(0.0, 2.0 * M_PI, n_theta, false);

    TriangleMesh mesh;
    mesh.vertices.reserve(n_z * n_theta);
    for (double z : zs) {
        double r = radius_base * (1.0 - (z + height / 2.0) / height);
        for (double phi : phis)
            mesh.vertices.push_back(cylindrical_to_cartesian(r, phi, z));
    }

    mesh.triangles = grid_triangles_closed_phi(n_z, n_theta);

    Polygon bottom;
    for (int j = 0; j < n_theta; ++j)
        bottom.push_back(grid_index(0, j, n_theta));
    for (const auto& t : triangulate_polygon(bottom)) mesh.triangles.push_back(t);

    return mesh;
}

/**
 * Torus centered at the origin, revolution axis z. Parametrization:
 *   x = (R + r*cos(v))*cos(u), y = (R + r*cos(v))*sin(u), z = r*sin(v)
 * with u (major angle) and v (minor angle) both in [0, 2*pi).
 * Not a coordinate-system conversion (unlike sphere/cylinder/cone):
 * this parametrization is specific to the torus shape, so it is
 * computed directly rather than routed through Coordinates.
 * Topology: torus_triangles — doubly periodic, no caps needed.
 * @param n1 points along u, the major direction (>= 3).
 * @param n2 points along v, the minor direction (>= 3).
 * @param R major radius: distance from the tube's center to the
 *          z-axis (default 1.0).
 * @param r minor radius: radius of the tube (default 0.3).
 *          Precondition: r < R, to avoid self-intersection.
 * @return a TriangleMesh with n1*n2 vertices.
 */
inline TriangleMesh torus(int n1, int n2, double R = 1.0, double r = 0.3) {
    auto us = linspace(0.0, 2.0 * M_PI, n1, false);
    auto vs = linspace(0.0, 2.0 * M_PI, n2, false);

    TriangleMesh mesh;
    mesh.vertices.reserve(n1 * n2);
    for (double u : us) {
        for (double v : vs) {
            double tube_radius = R + r * std::cos(v);
            mesh.vertices.push_back(Vec3{
                tube_radius * std::cos(u),
                tube_radius * std::sin(u),
                r * std::sin(v)
            });
        }
    }

    mesh.triangles = torus_triangles(n1, n2);
    return mesh;
}

}  // namespace raytracer::engine