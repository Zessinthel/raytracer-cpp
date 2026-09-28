// tests/test_engine.cpp
//
// Checks for the pure-math layer: vectors, matrices, coordinate conversions,
// grid generation and mesh connectivity. Links against `engine` only.
#include <array>
#include <numbers>

#include "check.hpp"
#include "raytracer/engine/coordinates.hpp"
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/topology.hpp"
#include "raytracer/engine/vec3.hpp"

using namespace raytracer::engine;
constexpr double pi = std::numbers::pi;

static void test_vec3() {
    Vec3 D{1.0, 2.0, 2.0};
    CHECK_NEAR(length_squared(D), 9.0, 1e-15);
    CHECK_NEAR(length(D), 3.0, 1e-15);
    CHECK_NEAR(length(normalized(D)), 1.0, 1e-15);

    Vec3 a{1.0, 2.0, 3.0};
    Vec3 b{4.0, -5.0, 6.0};
    CHECK_NEAR(dot(a, b), 12.0, 1e-15);
    CHECK_VEC(a + b, (Vec3{5.0, -3.0, 9.0}), 1e-15);
    CHECK_VEC(a - b, (Vec3{-3.0, 7.0, -3.0}), 1e-15);
    CHECK_VEC(2.0 * a, a * 2.0, 0.0);

    Vec3 ex{1.0, 0.0, 0.0}, ey{0.0, 1.0, 0.0}, ez{0.0, 0.0, 1.0};
    CHECK_VEC(cross(ex, ey), ez, 1e-15);
    CHECK_VEC(cross(ey, ez), ex, 1e-15);
    CHECK_VEC(cross(ez, ex), ey, 1e-15);
    CHECK_VEC(cross(a, b), cross(b, a) * -1.0, 1e-15);       // anticommutative
    CHECK_NEAR(dot(cross(a, b), a), 0.0, 1e-13);             // orthogonal to both factors
    CHECK_NEAR(dot(cross(a, b), b), 0.0, 1e-13);
}

static void test_mat3() {
    Vec3 X{2.0, 3.0, -1.0};
    CHECK_VEC(identity3() * X, X, 1e-15);
    CHECK_NEAR(identity3().determinant(), 1.0, 1e-15);

    Mat3 diag{{{2.0, 0.0, 0.0}, {0.0, 3.0, 0.0}, {0.0, 0.0, 4.0}}};
    CHECK_NEAR(diag.determinant(), 24.0, 1e-14);

    // Right-handed rotations: each takes the next axis of the cycle x -> y -> z -> x.
    Vec3 ex{1.0, 0.0, 0.0}, ey{0.0, 1.0, 0.0}, ez{0.0, 0.0, 1.0};
    CHECK_VEC(rotation_z(pi / 2.0) * ex, ey, 1e-12);
    CHECK_VEC(rotation_x(pi / 2.0) * ey, ez, 1e-12);
    CHECK_VEC(rotation_y(pi / 2.0) * ez, ex, 1e-12);

    // A rotation is orthogonal: its inverse is its transpose.
    Mat3 R = rotation_z(pi / 4.0);
    Mat3 Rinv = R.inverse();
    Mat3 Rt = R.transpose();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(Rinv.m[i][j], Rt.m[i][j], 1e-12);
    CHECK_NEAR(R.determinant(), 1.0, 1e-14);

    // inverse(A) * A == I for a generic invertible matrix (det = 3).
    Mat3 A{{{2.0, 0.0, 1.0}, {1.0, 3.0, 2.0}, {1.0, 0.0, 1.0}}};
    CHECK_NEAR(A.determinant(), 3.0, 1e-14);
    Mat3 product = A.inverse() * A;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK_NEAR(product.m[i][j], i == j ? 1.0 : 0.0, 1e-12);
}

static void test_coordinates() {
    CHECK_VEC(spherical_to_cartesian(2.0, pi / 2.0, 0.0), (Vec3{2.0, 0.0, 0.0}), 1e-12);
    CHECK_VEC(spherical_to_cartesian(1.0, 0.0, 1.234), (Vec3{0.0, 0.0, 1.0}), 1e-12);
    CHECK_VEC(spherical_to_cartesian(1.0, pi / 2.0, pi / 2.0), (Vec3{0.0, 1.0, 0.0}), 1e-12);
    CHECK_VEC(cylindrical_to_cartesian(2.0, pi / 2.0, 3.0), (Vec3{0.0, 2.0, 3.0}), 1e-12);
}

static void test_discretizer() {
    auto closed = linspace(0.0, 1.0, 5);
    CHECK(closed.size() == 5);
    CHECK_NEAR(closed.front(), 0.0, 0.0);
    CHECK_NEAR(closed.back(), 1.0, 1e-15);
    CHECK_NEAR(closed[1], 0.25, 1e-15);

    auto half_open = linspace(0.0, 1.0, 4, false);   // [0, 1): the end point is excluded
    CHECK(half_open.size() == 4);
    CHECK_NEAR(half_open.back(), 0.75, 1e-15);

    CHECK_NEAR(linspace(3.0, 9.0, 1).front(), 3.0, 0.0);

    auto grid = discretize_cartesian(2, 0.0, 1.0, 2, 0.0, 1.0, 1, 0.0, 0.0);
    CHECK(grid.size() == 4);
    CHECK_VEC(grid.front(), (Vec3{0.0, 0.0, 0.0}), 0.0);
    CHECK_VEC(grid.back(), (Vec3{1.0, 1.0, 0.0}), 0.0);

    auto ring = discretize_spherical(1, 1.0, 1.0, 3, 0.0, pi, 1, 0.0, 0.0);
    CHECK(ring.size() == 3);
    CHECK_VEC(ring[0], (Vec3{0.0, 0.0, 1.0}), 1e-12);    // north pole
    CHECK_VEC(ring[1], (Vec3{1.0, 0.0, 0.0}), 1e-12);    // equator
    CHECK_VEC(ring[2], (Vec3{0.0, 0.0, -1.0}), 1e-12);   // south pole
}

static void test_topology() {
    // The 3x3 grid of the theory notes: 4 quads, 8 triangles, each quad split
    // along its (i,j)-(i+1,j+1) diagonal.
    auto tris = grid_triangles_open(3, 3);
    std::array<Triangle, 8> expected = {{
        {0, 3, 4}, {0, 4, 1}, {1, 4, 5}, {1, 5, 2},
        {3, 6, 7}, {3, 7, 4}, {4, 7, 8}, {4, 8, 5}}};
    CHECK(tris.size() == 8);
    for (std::size_t k = 0; k < tris.size() && k < expected.size(); ++k)
        CHECK(tris[k] == expected[k]);

    CHECK(grid_triangles_open(4, 5).size() == 2u * 3 * 4);
    CHECK(grid_triangles_closed_phi(4, 6).size() == 2u * 3 * 6);
    CHECK(sphere_triangles(5, 6).size() == 2u * 4 * 6);
    CHECK(torus_triangles(5, 4).size() == 2u * 5 * 4);

    // Fan triangulation of an N-gon gives N-2 triangles sharing the first vertex.
    auto square = triangulate_polygon({0, 1, 2, 3});
    CHECK(square.size() == 2);
    CHECK(square[0] == (Triangle{0, 1, 2}));
    CHECK(square[1] == (Triangle{0, 2, 3}));
    CHECK(triangulate_polygon({0, 1, 2, 3, 4}).size() == 3);
}

int main() {
    test_vec3();
    test_mat3();
    test_coordinates();
    test_discretizer();
    test_topology();
    return check::report("engine");
}
