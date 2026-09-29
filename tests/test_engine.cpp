// tests/test_engine.cpp
//
// Checks for the pure-math layer: vectors, matrices, coordinate conversions,
// grid generation and mesh connectivity. Links against `engine` only.
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "check.hpp"
#include "raytracer/engine/coordinates.hpp"
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/flower.hpp"
#include "raytracer/engine/image.hpp"
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

    // reflect: mirroring (1,-1,0) off the horizontal plane (normal +y) flips
    // only the perpendicular (y) component, giving (1,1,0).
    CHECK_VEC(reflect(Vec3{1.0, -1.0, 0.0}, Vec3{0.0, 1.0, 0.0}), (Vec3{1.0, 1.0, 0.0}), 1e-15);
    // Reflecting straight into a surface bounces straight back.
    CHECK_VEC(reflect(Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}), (Vec3{0.0, 0.0, 1.0}), 1e-15);
    // A vector already in the mirror plane (perpendicular to n) is unchanged.
    CHECK_VEC(reflect(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 0.0, 1.0}), (Vec3{1.0, 0.0, 0.0}), 1e-15);
    // Length is preserved, and reflecting twice restores the original vector,
    // for several unrelated, non-axis-aligned (d, n) pairs.
    for (const auto& [d, n] : {std::pair{Vec3{2.0, -3.0, 5.0}, normalized(Vec3{1.0, 2.0, 2.0})},
                               std::pair{Vec3{-1.0, 0.5, 4.0}, normalized(Vec3{0.0, 3.0, 4.0})},
                               std::pair{Vec3{7.0, 7.0, -2.0}, normalized(Vec3{-1.0, 1.0, 1.0})}}) {
        Vec3 r = reflect(d, n);
        CHECK_NEAR(length(r), length(d), 1e-12);
        CHECK_VEC(reflect(r, n), d, 1e-12);
        // The component along n flips sign; the component perpendicular to n
        // is unchanged (2.3.2's decomposition d = d_par + d_perp).
        CHECK_NEAR(dot(r, n), -dot(d, n), 1e-12);
        Vec3 d_perp = d - n * dot(d, n);
        Vec3 r_perp = r - n * dot(r, n);
        CHECK_VEC(r_perp, d_perp, 1e-12);
    }
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

static void test_image() {
    Image image(3, 2);
    CHECK(image.width == 3 && image.height == 2);
    CHECK(image.pixels.size() == 6);

    // A new image is black.
    for (const Color& c : image.pixels)
        CHECK(c.r == 0.0 && c.g == 0.0 && c.b == 0.0);

    // Row-major with row 0 at the top: pixel (x = 2, y = 1) is the last one.
    image.at(2, 1) = Color{0.1, 0.2, 0.3};
    image.at(0, 0) = Color{0.4, 0.5, 0.6};
    CHECK_NEAR(image.pixels[5].g, 0.2, 0.0);
    CHECK_NEAR(image.pixels[0].b, 0.6, 0.0);
    const Image& view = image;
    CHECK_NEAR(view.at(2, 1).r, 0.1, 0.0);

    // Non-positive dimensions are rejected before anything is allocated.
    int rejected = 0;
    for (auto [w, h] : {std::pair{0, 4}, std::pair{4, 0}, std::pair{-1, 4}}) {
        try { Image bad(w, h); } catch (const std::invalid_argument&) { ++rejected; }
    }
    CHECK(rejected == 3);
}

static void test_color() {
    using raytracer::engine::Color;
    using raytracer::engine::saturate;

    Color a{0.2, 0.5, 0.8};
    CHECK_NEAR((a * 2.0).r, 0.4, 1e-15);
    CHECK_NEAR((2.0 * a).b, 1.6, 1e-15);
    Color b{0.1, 0.1, 0.1};
    Color sum = a + b;
    CHECK_NEAR(sum.r, 0.3, 1e-15);
    CHECK_NEAR(sum.g, 0.6, 1e-15);
    CHECK_NEAR(sum.b, 0.9, 1e-15);

    // saturate clamps each channel independently, in either direction.
    Color out_of_range{-0.5, 0.5, 1.5};
    Color clamped = saturate(out_of_range);
    CHECK_NEAR(clamped.r, 0.0, 0.0);
    CHECK_NEAR(clamped.g, 0.5, 0.0);
    CHECK_NEAR(clamped.b, 1.0, 0.0);

    // A value already inside [0, 1] is left alone.
    Color inside{0.0, 0.3, 1.0};
    Color still = saturate(inside);
    CHECK_NEAR(still.r, 0.0, 0.0);
    CHECK_NEAR(still.g, 0.3, 0.0);
    CHECK_NEAR(still.b, 1.0, 0.0);

    // A NaN channel clamps to 0, like io::to_byte: a bug upstream should show
    // up as black, not propagate as an arbitrary value.
    Color with_nan{std::nan(""), 0.5, 0.5};
    CHECK_NEAR(saturate(with_nan).r, 0.0, 0.0);
}

static void test_floor_mod() {
    using raytracer::engine::detail::floor_mod;
    // Unlike std::fmod, floor_mod always returns a value with the sign of
    // the divisor: for a negative dividend the two disagree completely, not
    // just at a rounding boundary. flower()'s petal envelope depends on
    // this: its domain starts at a negative s, and std::fmod there would
    // distort the petals nearest the center.
    CHECK_NEAR(floor_mod(-7.2, 2.0 * pi), 5.36637061436, 1e-9);   // numpy.mod(-7.2, 2*pi), verified independently
    CHECK_NEAR(floor_mod(7.2, 2.0 * pi), 0.916814692820, 1e-9);    // a positive dividend: same magnitude either way
    CHECK_NEAR(floor_mod(0.0, 2.0 * pi), 0.0, 1e-12);
    CHECK_NEAR(floor_mod(2.0 * pi, 2.0 * pi), 0.0, 1e-9);         // an exact multiple wraps to 0, not n
    CHECK_NEAR(floor_mod(-2.0 * pi, 2.0 * pi), 0.0, 1e-9);
    for (double a : {-100.0, -1.0, 1.0, 100.0}) {
        double r = floor_mod(a, 2.0 * pi);
        CHECK(r >= 0.0 && r < 2.0 * pi);   // always in [0, n), whatever the sign of a
    }
}

int main() {
    test_vec3();
    test_mat3();
    test_coordinates();
    test_discretizer();
    test_topology();
    test_image();
    test_color();
    test_floor_mod();
    return check::report("engine");
}
