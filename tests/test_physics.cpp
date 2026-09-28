// tests/test_physics.cpp
//
// Checks for the ray-triangle and ray-mesh intersection: Moller-Trumbore on a
// single triangle, the search interval [t_min, t_max), degenerate-triangle
// filtering, and the (intrinsic, never flipped) normal reported for each
// primitive. Links against `physics` and `engine`.
#include <cmath>
#include <optional>
#include <random>

#include "check.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/physics/triangle.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;
using raytracer::physics::MeshHit;
using raytracer::physics::intersect_triangle;

static void test_triangle() {
    Vec3 v0{-1.0, -1.0, 0.0};
    Vec3 v1{1.0, -1.0, 0.0};
    Vec3 v2{0.0, 1.0, 0.0};

    // A ray at the centroid, perpendicular to the triangle: t = 1, and the
    // barycentric coordinates reproduce the hit point (0, 0, 0).
    auto hit = intersect_triangle(Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2, 0.0, T_INFINITE);
    CHECK(hit.has_value());
    if (hit) {
        CHECK_NEAR(hit->t, 1.0, 1e-12);
        CHECK_NEAR(hit->u, 0.25, 1e-12);
        CHECK_NEAR(hit->v, 0.5, 1e-12);
    }

    // Parallel to the plane of the triangle: no intersection.
    CHECK(!intersect_triangle(Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, v0, v1, v2, 0.0, T_INFINITE).has_value());
    // Passing outside the triangle.
    CHECK(!intersect_triangle(Vec3{5.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2, 0.0, T_INFINITE).has_value());
    // The triangle is behind the ray origin (t < 0).
    CHECK(!intersect_triangle(Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2, 0.0, T_INFINITE).has_value());
}

static void test_triangle_interval() {
    Vec3 v0{-1.0, -1.0, 0.0};
    Vec3 v1{1.0, -1.0, 0.0};
    Vec3 v2{0.0, 1.0, 0.0};
    Vec3 O{0.0, 0.0, -1.0};
    Vec3 D{0.0, 0.0, 1.0};

    // For these values the arithmetic is exact and the hit is at t = 1 to the
    // last bit, so the half-open boundaries can be tested without tolerance.
    CHECK(intersect_triangle(O, D, v0, v1, v2, 0.0, 1.5).has_value());
    CHECK(!intersect_triangle(O, D, v0, v1, v2, 0.0, 1.0).has_value());     // upper end excluded
    CHECK(!intersect_triangle(O, D, v0, v1, v2, 0.0, 0.5).has_value());
    CHECK(intersect_triangle(O, D, v0, v1, v2, 1.0, T_INFINITE).has_value());   // lower end included
    CHECK(!intersect_triangle(O, D, v0, v1, v2, 1.5, T_INFINITE).has_value());
    CHECK(intersect_triangle(O, D, v0, v1, v2, 0.5, 1.5).has_value());

    // Hits behind the origin stay excluded whatever t_max is.
    CHECK(!intersect_triangle(Vec3{0.0, 0.0, 1.0}, D, v0, v1, v2, 0.0, T_INFINITE).has_value());
    // A NaN direction produces a NaN t, which must be rejected, not accepted.
    CHECK(!intersect_triangle(O, Vec3{0.0, 0.0, std::nan("")}, v0, v1, v2, 0.0, T_INFINITE).has_value());
}

static void test_mesh_construction() {
    // sphere(5, 6) has 48 triangles; 12 of them, one per quad touching each
    // pole, have zero area and are removed. All 30 vertices are kept.
    Mesh coarse = Mesh::from_triangle_mesh(sphere(5, 6, 1.0));
    CHECK(coarse.vertex_count() == 30);
    CHECK(coarse.triangle_count() == 36);

    // Primitives without degeneracies keep every triangle.
    CHECK(Mesh::from_triangle_mesh(cube(1.0)).triangle_count() == 12);
    CHECK(Mesh::from_triangle_mesh(star()).triangle_count() == 20);
}

static void test_mesh_hits() {
    // A ray displaced from the axis of a fine sphere reproduces the analytic
    // distance 5 - sqrt(1 - 0.1^2), up to the tessellation error.
    Mesh fine = Mesh::from_triangle_mesh(sphere(64, 64, 1.0));
    Ray offset{Vec3{0.1, 0.0, -5.0}, Vec3{0.0, 0.0, 1.0}};
    auto hit = fine.intersect(offset, 0.0, T_INFINITE);
    CHECK(hit.has_value());
    if (hit) {
        CHECK_NEAR(hit->t, 5.0 - std::sqrt(0.99), 2e-3);
        Vec3 point = offset.at(hit->t);
        CHECK(dot(hit->normal, normalized(point)) > 0.99);   // radial and outward
    }

    // A ray that leaves the mesh behind it finds nothing.
    CHECK(!fine.intersect(Ray{Vec3{0.0, 0.0, -5.0}, Vec3{0.0, 0.0, -1.0}}, 0.0, T_INFINITE).has_value());
}

static void test_mesh_interval() {
    // A ray along +z through the cube of side 2 enters through the bottom
    // face at t = 4 and leaves through the top face at t = 6.
    Mesh box = Mesh::from_triangle_mesh(cube(2.0));
    Ray ray{Vec3{0.3, 0.2, -5.0}, Vec3{0.0, 0.0, 1.0}};

    auto entry = box.intersect(ray, 0.0, T_INFINITE);
    CHECK(entry.has_value());
    if (entry) {
        CHECK_NEAR(entry->t, 4.0, 1e-12);
        CHECK_VEC(entry->normal, (Vec3{0.0, 0.0, -1.0}), 1e-12);
    }

    // Upper end below the entry: nothing. Above it: the entry, not the exit.
    CHECK(!box.intersect(ray, 0.0, 3.5).has_value());
    auto still_entry = box.intersect(ray, 0.0, 4.5);
    CHECK(still_entry.has_value());
    if (still_entry) CHECK_NEAR(still_entry->t, 4.0, 1e-12);

    // Lower end past the entry skips it and finds the exit through the top,
    // whose outward normal is +z. This is how a secondary ray leaves the
    // surface it starts on.
    auto exit_hit = box.intersect(ray, 4.5, T_INFINITE);
    CHECK(exit_hit.has_value());
    if (exit_hit) {
        CHECK_NEAR(exit_hit->t, 6.0, 1e-12);
        CHECK_VEC(exit_hit->normal, (Vec3{0.0, 0.0, 1.0}), 1e-12);
    }
    CHECK(!box.intersect(ray, 6.5, T_INFINITE).has_value());
}

// Independent reference: every triangle is tested against the full interval,
// with no narrowing, and the smallest t wins (first triangle on a tie).
static std::optional<MeshHit> brute_force(const TriangleMesh& raw, const Ray& ray,
                                          double t_min, double t_max) {
    std::optional<MeshHit> best;
    for (std::size_t i = 0; i < raw.triangles.size(); ++i) {
        const auto& tri = raw.triangles[i];
        const Vec3& v0 = raw.vertices[tri[0]];
        const Vec3& v1 = raw.vertices[tri[1]];
        const Vec3& v2 = raw.vertices[tri[2]];
        auto hit = intersect_triangle(ray.origin, ray.direction, v0, v1, v2, t_min, t_max);
        if (hit && (!best || hit->t < best->t))
            best = MeshHit{hit->t, normalized(cross(v1 - v0, v2 - v0)), static_cast<int>(i)};
    }
    return best;
}

static void test_narrowing_matches_brute_force() {
    // None of these meshes has degenerate triangles, so indices in the Mesh
    // and in the raw triangle list coincide.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> far_point(-4.0, 4.0);
    std::uniform_real_distribution<double> near_point(-1.3, 1.3);

    const TriangleMesh shapes[] = {torus(48, 24, 1.0, 0.3), star(), cube(2.0)};
    const double intervals[][2] = {{0.0, T_INFINITE}, {2.0, 6.0}, {0.0, 3.0}};

    for (const TriangleMesh& raw : shapes) {
        Mesh mesh = Mesh::from_triangle_mesh(raw);
        CHECK(mesh.triangle_count() == raw.triangles.size());

        int rays_that_hit = 0, mismatches = 0;
        for (const auto& interval : intervals) {
            for (int k = 0; k < 700; ++k) {
                Vec3 origin{far_point(rng), far_point(rng), far_point(rng)};
                Vec3 target{near_point(rng), near_point(rng), near_point(rng)};
                Ray ray{origin, normalized(target - origin)};

                auto expected = brute_force(raw, ray, interval[0], interval[1]);
                auto actual = mesh.intersect(ray, interval[0], interval[1]);

                if (expected.has_value() != actual.has_value()) { ++mismatches; continue; }
                if (!expected) continue;
                ++rays_that_hit;
                if (expected->t != actual->t || expected->triangle_index != actual->triangle_index ||
                    expected->normal.x != actual->normal.x || expected->normal.y != actual->normal.y ||
                    expected->normal.z != actual->normal.z)
                    ++mismatches;
            }
        }
        CHECK(mismatches == 0);
        CHECK(rays_that_hit > 80);     // the comparison is not vacuous (measured: 234 to 1446)
    }
}

static void test_mesh_occluded() {
    // The cube along the ray: entry at t = 4, exit at t = 6.
    Mesh box = Mesh::from_triangle_mesh(cube(2.0));
    Ray ray{Vec3{0.3, 0.2, -5.0}, Vec3{0.0, 0.0, 1.0}};
    CHECK(box.occluded(ray, 0.0, T_INFINITE));
    CHECK(!box.occluded(ray, 0.0, 3.5));            // interval ends before the cube
    CHECK(box.occluded(ray, 0.0, 4.5));             // it contains the entry
    CHECK(box.occluded(ray, 4.5, T_INFINITE));      // it contains only the exit
    CHECK(!box.occluded(ray, 6.5, T_INFINITE));     // interval starts after the cube
    CHECK(!box.occluded(Ray{Vec3{0.3, 0.2, -5.0}, Vec3{0.0, 0.0, -1.0}}, 0.0, T_INFINITE));

    // occluded() must agree with "intersect() finds something" for any
    // interval, on shapes with and without concavities.
    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> far_point(-4.0, 4.0);
    std::uniform_real_distribution<double> near_point(-1.3, 1.3);
    std::uniform_real_distribution<double> lower(0.0, 6.0);
    std::uniform_real_distribution<double> width(0.0, 4.0);

    const TriangleMesh shapes[] = {torus(48, 24, 1.0, 0.3), star(), cube(2.0)};
    for (const TriangleMesh& raw : shapes) {
        Mesh mesh = Mesh::from_triangle_mesh(raw);
        int yes = 0, no = 0, mismatches = 0;
        for (int k = 0; k < 1500; ++k) {
            Vec3 origin{far_point(rng), far_point(rng), far_point(rng)};
            Vec3 target{near_point(rng), near_point(rng), near_point(rng)};
            Ray r{origin, normalized(target - origin)};
            double t_min = lower(rng);
            double t_max = (k % 4 == 0) ? T_INFINITE : t_min + width(rng);
            bool blocked = mesh.occluded(r, t_min, t_max);
            if (blocked != mesh.intersect(r, t_min, t_max).has_value())
                ++mismatches;
            (blocked ? yes : no)++;
        }
        CHECK(mismatches == 0);
        CHECK(yes > 40 && no > 40);    // both outcomes are exercised (measured: at least 103 and 863)
    }
}

static void test_normals_are_intrinsic() {
    // The normal follows the winding, not the ray: seen from outside the
    // bottom face of the cube reports -z, and the top face seen from inside
    // reports +z, the direction it faces outward.
    Mesh box = Mesh::from_triangle_mesh(cube(2.0));

    auto from_below = box.intersect(Ray{Vec3{0.3, 0.2, -5.0}, Vec3{0.0, 0.0, 1.0}}, 0.0, T_INFINITE);
    CHECK(from_below.has_value());
    if (from_below) {
        CHECK_NEAR(from_below->t, 4.0, 1e-12);
        CHECK_VEC(from_below->normal, (Vec3{0.0, 0.0, -1.0}), 1e-12);
    }

    auto from_inside = box.intersect(Ray{Vec3{0.3, 0.2, 0.0}, Vec3{0.0, 0.0, 1.0}}, 0.0, T_INFINITE);
    CHECK(from_inside.has_value());
    if (from_inside) {
        CHECK_NEAR(from_inside->t, 1.0, 1e-12);
        CHECK_VEC(from_inside->normal, (Vec3{0.0, 0.0, 1.0}), 1e-12);
    }
}

static void test_outward_normals_of_primitives() {
    // Bottom cap of the cylinder, seen from below: outward is -z.
    Mesh can = Mesh::from_triangle_mesh(cylinder(10, 32, 0.8, 2.0));
    auto cap = can.intersect(Ray{Vec3{0.13, 0.21, -5.0}, Vec3{0.0, 0.0, 1.0}}, 0.0, T_INFINITE);
    CHECK(cap.has_value());
    if (cap) {
        CHECK_NEAR(cap->t, 4.0, 1e-12);
        CHECK_VEC(cap->normal, (Vec3{0.0, 0.0, -1.0}), 1e-12);
    }

    // Torus (R = 1, r = 0.3): from far away on +x the outer side faces +x...
    Mesh ring = Mesh::from_triangle_mesh(torus(64, 32, 1.0, 0.3));
    auto outer = ring.intersect(Ray{Vec3{5.0, 0.1, 0.05}, Vec3{-1.0, 0.0, 0.0}}, 0.0, T_INFINITE);
    CHECK(outer.has_value());
    if (outer) CHECK(outer->normal.x > 0.9);

    // ...and from inside the hole, looking toward -x, the inner side of the
    // tube faces the axis, which is +x there. A torus wound inward would
    // fail both checks.
    auto inner = ring.intersect(Ray{Vec3{0.0, 0.1, 0.05}, Vec3{-1.0, 0.0, 0.0}}, 0.0, T_INFINITE);
    CHECK(inner.has_value());
    if (inner) CHECK(inner->normal.x > 0.9);

    // Cone, from the side at mid height: the lateral normal points outward
    // and up, since the surface narrows toward the apex.
    Mesh spike = Mesh::from_triangle_mesh(cone(12, 64, 1.0, 2.0));
    auto side = spike.intersect(Ray{Vec3{5.0, 0.03, 0.0}, Vec3{-1.0, 0.0, 0.0}}, 0.0, T_INFINITE);
    CHECK(side.has_value());
    if (side) {
        CHECK(side->normal.x > 0.5);
        CHECK(side->normal.z > 0.3);
    }
}

int main() {
    test_triangle();
    test_triangle_interval();
    test_mesh_construction();
    test_mesh_hits();
    test_mesh_interval();
    test_narrowing_matches_brute_force();
    test_mesh_occluded();
    test_normals_are_intrinsic();
    test_outward_normals_of_primitives();
    return check::report("physics");
}
