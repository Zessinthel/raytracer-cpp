// include/raytracer/engine/topology.hpp
#pragma once
#include <vector>
#include <array>

namespace raytracer::engine {

using Triangle = std::array<int, 3>;
using TriangleList = std::vector<Triangle>;
using Polygon = std::vector<int>;

/**
 * Linear vertex index for a point in a row-major (n1 x n2) grid.
 *   index(i, j) = i * n2 + j
 * Matches the layout produced by Discretizer's nested loops.
 * @param i index along the first dimension.
 * @param j index along the second dimension.
 * @param n2 size of the second dimension.
 * @return the flat vertex index.
 */
inline int grid_index(int i, int j, int n2) {
    return i * n2 + j;
}

/**
 * Fan triangulation of a convex polygon given by its vertex indices:
 * splits an N-gon into (N-2) triangles, all sharing the polygon's
 * first vertex. Precondition: polygon must be convex.
 * @param polygon vertex indices of a convex polygon (size >= 3).
 * @return (size-2) triangles.
 */
inline TriangleList triangulate_polygon(const Polygon& polygon) {
    TriangleList triangles;
    for (std::size_t i = 1; i + 1 < polygon.size(); ++i) {
        triangles.push_back({polygon[0], polygon[i], polygon[i + 1]});
    }
    return triangles;
}

/**
 * Adds the two triangles covering the quad with corners
 * (i,j), (i+1,j), (i+1,j+1), (i,j+1), split along the (i,j)-(i+1,j+1)
 * diagonal, to the given list.
 * @param triangles list to append to.
 * @param i0, j0 first grid index of the quad.
 * @param i1, j1 second grid index of the quad (i0/j0's neighbor).
 * @param n2 size of the second grid dimension (for index math).
 */
inline void append_quad_triangles(TriangleList& triangles,
                                   int i0, int j0, int i1, int j1, int n2) {
    int a = grid_index(i0, j0, n2);
    int b = grid_index(i1, j0, n2);
    int c = grid_index(i1, j1, n2);
    int d = grid_index(i0, j1, n2);
    triangles.push_back({a, b, c});
    triangles.push_back({a, c, d});
}

/**
 * Triangulated topology for an open (n1 x n2) grid, no periodicity
 * in either direction. Covers flat planes/rectangles, Cartesian
 * domains, and any parametric surface without periodic boundaries.
 * @param n1 points along the first direction (>= 2).
 * @param n2 points along the second direction (>= 2).
 * @return 2*(n1-1)*(n2-1) triangles.
 */
inline TriangleList grid_triangles_open(int n1, int n2) {
    TriangleList triangles;
    for (int i = 0; i < n1 - 1; ++i)
        for (int j = 0; j < n2 - 1; ++j)
            append_quad_triangles(triangles, i, j, i + 1, j + 1, n2);
    return triangles;
}

/**
 * Triangulated topology for a grid closed (periodic) in the second
 * direction, open in the first. Covers cylinder/cone lateral bodies,
 * surfaces of revolution without degeneracy at the ends.
 * @param n1 points along the open direction (>= 2).
 * @param n2 points along the periodic direction (>= 3).
 * @return 2*(n1-1)*n2 triangles.
 */
inline TriangleList grid_triangles_closed_phi(int n1, int n2) {
    TriangleList triangles;
    for (int i = 0; i < n1 - 1; ++i) {
        for (int j = 0; j < n2; ++j) {
            int j_next = (j + 1) % n2;
            int a = grid_index(i,   j,      n2);
            int b = grid_index(i,   j_next, n2);
            int c = grid_index(i+1, j_next, n2);
            int d = grid_index(i+1, j,      n2);
            triangles.push_back({a, b, c});
            triangles.push_back({a, c, d});
        }
    }
    return triangles;
}

/**
 * Triangulated spherical topology: closed_phi body plus fan-triangulated
 * caps at the two poles, where the angular grid degenerates to a point.
 * @param n_theta points in colatitude, including poles (>= 3).
 * @param n_phi points in azimuth (>= 3).
 * @return 2*(n_theta-2)*n_phi body triangles + 2*(n_phi-2) cap triangles.
 */
inline TriangleList sphere_triangles(int n_theta, int n_phi) {
    TriangleList triangles;

    Polygon north;
    for (int j = 0; j < n_phi; ++j) north.push_back(grid_index(0, j, n_phi));
    for (const auto& t : triangulate_polygon(north)) triangles.push_back(t);

    for (int i = 0; i < n_theta - 1; ++i) {
        for (int j = 0; j < n_phi; ++j) {
            int j_next = (j + 1) % n_phi;
            int a = grid_index(i,   j,      n_phi);
            int b = grid_index(i,   j_next, n_phi);
            int c = grid_index(i+1, j_next, n_phi);
            int d = grid_index(i+1, j,      n_phi);
            triangles.push_back({a, b, c});
            triangles.push_back({a, c, d});
        }
    }

    Polygon south;
    for (int j = 0; j < n_phi; ++j) south.push_back(grid_index(n_theta - 1, j, n_phi));
    for (const auto& t : triangulate_polygon(south)) triangles.push_back(t);

    return triangles;
}

/**
 * Triangulated toroidal topology: periodic in both directions.
 * @param n1 points along the first direction (>= 3).
 * @param n2 points along the second direction (>= 3).
 * @return 2*n1*n2 triangles, doubly periodic.
 */
inline TriangleList torus_triangles(int n1, int n2) {
    TriangleList triangles;
    for (int i = 0; i < n1; ++i) {
        int i_next = (i + 1) % n1;
        for (int j = 0; j < n2; ++j) {
            int j_next = (j + 1) % n2;
            int a = grid_index(i,      j,      n2);
            int b = grid_index(i,      j_next, n2);
            int c = grid_index(i_next, j_next, n2);
            int d = grid_index(i_next, j,      n2);
            triangles.push_back({a, b, c});
            triangles.push_back({a, c, d});
        }
    }
    return triangles;
}

}  // namespace raytracer::engine