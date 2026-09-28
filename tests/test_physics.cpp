// tests/test_physics.cpp
//
// Checks for the ray-triangle and ray-mesh intersection: Moller-Trumbore on a
// single triangle, degenerate-triangle filtering, and the (intrinsic, never
// flipped) normal reported for each primitive. Links against `physics` and
// `engine`.
#include <cmath>

#include "check.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/physics/triangle.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;
using raytracer::physics::intersect_triangle;

static void test_triangle() {
    Vec3 v0{-1.0, -1.0, 0.0};
    Vec3 v1{1.0, -1.0, 0.0};
    Vec3 v2{0.0, 1.0, 0.0};

    // A ray at the centroid, perpendicular to the triangle: t = 1, and the
    // barycentric coordinates reproduce the hit point (0, 0, 0).
    auto hit = intersect_triangle(Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2);
    CHECK(hit.has_value());
    if (hit) {
        CHECK_NEAR(hit->t, 1.0, 1e-12);
        CHECK_NEAR(hit->u, 0.25, 1e-12);
        CHECK_NEAR(hit->v, 0.5, 1e-12);
    }

    // Parallel to the plane of the triangle: no intersection.
    CHECK(!intersect_triangle(Vec3{0.0, 0.0, 0.0}, Vec3{1.0, 0.0, 0.0}, v0, v1, v2).has_value());
    // Passing outside the triangle.
    CHECK(!intersect_triangle(Vec3{5.0, 0.0, -1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2).has_value());
    // The triangle is behind the ray origin (t < 0).
    CHECK(!intersect_triangle(Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 1.0}, v0, v1, v2).has_value());
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
    auto hit = fine.intersect(offset);
    CHECK(hit.has_value());
    if (hit) {
        CHECK_NEAR(hit->t, 5.0 - std::sqrt(0.99), 2e-3);
        Vec3 point = offset.at(hit->t);
        CHECK(dot(hit->normal, normalized(point)) > 0.99);   // radial and outward
    }

    // A ray that leaves the mesh behind it finds nothing.
    CHECK(!fine.intersect(Ray{Vec3{0.0, 0.0, -5.0}, Vec3{0.0, 0.0, -1.0}}).has_value());
}

static void test_normals_are_intrinsic() {
    // The normal follows the winding, not the ray: seen from outside the
    // bottom face of the cube reports -z, and the top face seen from inside
    // reports +z, the direction it faces outward.
    Mesh box = Mesh::from_triangle_mesh(cube(2.0));

    auto from_below = box.intersect(Ray{Vec3{0.3, 0.2, -5.0}, Vec3{0.0, 0.0, 1.0}});
    CHECK(from_below.has_value());
    if (from_below) {
        CHECK_NEAR(from_below->t, 4.0, 1e-12);
        CHECK_VEC(from_below->normal, (Vec3{0.0, 0.0, -1.0}), 1e-12);
    }

    auto from_inside = box.intersect(Ray{Vec3{0.3, 0.2, 0.0}, Vec3{0.0, 0.0, 1.0}});
    CHECK(from_inside.has_value());
    if (from_inside) {
        CHECK_NEAR(from_inside->t, 1.0, 1e-12);
        CHECK_VEC(from_inside->normal, (Vec3{0.0, 0.0, 1.0}), 1e-12);
    }
}

static void test_outward_normals_of_primitives() {
    // Bottom cap of the cylinder, seen from below: outward is -z.
    Mesh can = Mesh::from_triangle_mesh(cylinder(10, 32, 0.8, 2.0));
    auto cap = can.intersect(Ray{Vec3{0.13, 0.21, -5.0}, Vec3{0.0, 0.0, 1.0}});
    CHECK(cap.has_value());
    if (cap) {
        CHECK_NEAR(cap->t, 4.0, 1e-12);
        CHECK_VEC(cap->normal, (Vec3{0.0, 0.0, -1.0}), 1e-12);
    }

    // Torus (R = 1, r = 0.3): from far away on +x the outer side faces +x...
    Mesh ring = Mesh::from_triangle_mesh(torus(64, 32, 1.0, 0.3));
    auto outer = ring.intersect(Ray{Vec3{5.0, 0.1, 0.05}, Vec3{-1.0, 0.0, 0.0}});
    CHECK(outer.has_value());
    if (outer) CHECK(outer->normal.x > 0.9);

    // ...and from inside the hole, looking toward -x, the inner side of the
    // tube faces the axis, which is +x there. A torus wound inward would
    // fail both checks.
    auto inner = ring.intersect(Ray{Vec3{0.0, 0.1, 0.05}, Vec3{-1.0, 0.0, 0.0}});
    CHECK(inner.has_value());
    if (inner) CHECK(inner->normal.x > 0.9);

    // Cone, from the side at mid height: the lateral normal points outward
    // and up, since the surface narrows toward the apex.
    Mesh spike = Mesh::from_triangle_mesh(cone(12, 64, 1.0, 2.0));
    auto side = spike.intersect(Ray{Vec3{5.0, 0.03, 0.0}, Vec3{-1.0, 0.0, 0.0}});
    CHECK(side.has_value());
    if (side) {
        CHECK(side->normal.x > 0.5);
        CHECK(side->normal.z > 0.3);
    }
}

int main() {
    test_triangle();
    test_mesh_construction();
    test_mesh_hits();
    test_normals_are_intrinsic();
    test_outward_normals_of_primitives();
    return check::report("physics");
}
