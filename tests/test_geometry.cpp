// tests/test_geometry.cpp
//
// Checks for the mesh generators of the engine layer: every closed primitive
// must be a closed surface (each edge shared by exactly two triangles) with
// outward-facing normals and the volume of the solid it discretizes.
// Links against `engine` only.
#include <array>
#include <cmath>
#include <map>
#include <numbers>
#include <utility>
#include <vector>

#include "check.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/transform.hpp"
#include "raytracer/engine/vec3.hpp"

using namespace raytracer::engine;
constexpr double pi = std::numbers::pi;

// Signed volume by the divergence theorem, V = (1/6) * sum of dot(v0, cross(v1, v2)).
// It is positive for a closed surface whose triangles face outward and
// negative if they face inward, and it does not depend on the origin.
static double signed_volume(const TriangleMesh& mesh) {
    double sum = 0.0;
    for (const auto& tri : mesh.triangles) {
        const Vec3& v0 = mesh.vertices[tri[0]];
        const Vec3& v1 = mesh.vertices[tri[1]];
        const Vec3& v2 = mesh.vertices[tri[2]];
        sum += dot(v0, cross(v1, v2));
    }
    return sum / 6.0;
}

// Number of undirected edges that are not shared by exactly two triangles.
// Vertices at the same position (the collapsed rings at the poles of a sphere
// or the apex of a cone have distinct indices) are welded first, and the
// zero-area triangles that result are ignored.
static int non_manifold_edges(const TriangleMesh& mesh) {
    std::map<std::array<long long, 3>, int> welded;
    std::vector<int> id;
    id.reserve(mesh.vertices.size());
    for (const Vec3& v : mesh.vertices) {
        std::array<long long, 3> key{std::llround(v.x * 1e9),
                                     std::llround(v.y * 1e9),
                                     std::llround(v.z * 1e9)};
        auto it = welded.find(key);
        if (it == welded.end())
            it = welded.emplace(key, static_cast<int>(welded.size())).first;
        id.push_back(it->second);
    }

    std::map<std::pair<int, int>, int> uses;
    for (const auto& tri : mesh.triangles) {
        int a = id[tri[0]], b = id[tri[1]], c = id[tri[2]];
        if (a == b || b == c || a == c)
            continue;
        ++uses[{std::min(a, b), std::max(a, b)}];
        ++uses[{std::min(b, c), std::max(b, c)}];
        ++uses[{std::min(c, a), std::max(c, a)}];
    }

    int bad = 0;
    for (const auto& entry : uses)
        if (entry.second != 2)
            ++bad;
    return bad;
}

// Triangles whose normal points toward an interior point of a convex solid.
static int inward_faces(const TriangleMesh& mesh, const Vec3& interior) {
    int count = 0;
    for (const auto& tri : mesh.triangles) {
        const Vec3& v0 = mesh.vertices[tri[0]];
        const Vec3& v1 = mesh.vertices[tri[1]];
        const Vec3& v2 = mesh.vertices[tri[2]];
        Vec3 normal = cross(v1 - v0, v2 - v0);
        Vec3 centroid = (v0 + v1 + v2) * (1.0 / 3.0);
        if (dot(normal, centroid - interior) < -1e-12)
            ++count;
    }
    return count;
}

static void test_closed_primitives() {
    const Vec3 origin{0.0, 0.0, 0.0};

    auto s = sphere(32, 64, 1.0);
    CHECK(non_manifold_edges(s) == 0);
    CHECK(inward_faces(s, origin) == 0);
    CHECK_NEAR(signed_volume(s) / (4.0 / 3.0 * pi), 1.0, 0.01);

    auto c = cube(2.0);
    CHECK(non_manifold_edges(c) == 0);
    CHECK(inward_faces(c, origin) == 0);
    CHECK_NEAR(signed_volume(c), 8.0, 1e-12);

    // Regular tetrahedron of circumradius R: volume 8*sqrt(3)/27 * R^3.
    auto t = tetrahedron(1.0);
    CHECK(non_manifold_edges(t) == 0);
    CHECK(inward_faces(t, origin) == 0);
    CHECK_NEAR(signed_volume(t), 8.0 * std::sqrt(3.0) / 27.0, 1e-12);

    // Cylinder of radius 1 and height 2: both caps must face outward, which
    // the signed volume detects (a flipped cap would change it by a third).
    auto y = cylinder(8, 64, 1.0, 2.0);
    CHECK(non_manifold_edges(y) == 0);
    CHECK(inward_faces(y, origin) == 0);
    CHECK_NEAR(signed_volume(y) / (pi * 1.0 * 1.0 * 2.0), 1.0, 0.005);

    // Cone of base radius 1 and height 2, interior point a quarter of the way up.
    auto k = cone(12, 64, 1.0, 2.0);
    CHECK(non_manifold_edges(k) == 0);
    CHECK(inward_faces(k, Vec3{0.0, 0.0, -0.5}) == 0);
    CHECK_NEAR(signed_volume(k) / (pi * 1.0 * 1.0 * 2.0 / 3.0), 1.0, 0.005);

    // Torus with R = 1, r = 0.3: V = 2 pi^2 R r^2. Not convex, so orientation
    // is judged by the sign of the volume.
    auto o = torus(64, 32, 1.0, 0.3);
    CHECK(non_manifold_edges(o) == 0);
    CHECK(signed_volume(o) > 0.0);
    CHECK_NEAR(signed_volume(o) / (2.0 * pi * pi * 1.0 * 0.3 * 0.3), 1.0, 0.02);
}

static void test_star() {
    // Five points, R = 1, inner radius ratio of the regular pentagram, half
    // thickness 0.3. Area of the planar star = n R r sin(pi/n); the solid is
    // two pyramids over it, so V = 2 * area * h / 3, exactly.
    auto st = star();
    double r = 0.381966;
    double area = 5.0 * 1.0 * r * std::sin(pi / 5.0);
    CHECK(st.vertices.size() == 12);
    CHECK(st.triangles.size() == 20);
    CHECK(non_manifold_edges(st) == 0);
    CHECK_NEAR(signed_volume(st), 2.0 * area * 0.3 / 3.0, 1e-9);
}

static void test_plane() {
    // A plane must face +z, so that it works as a floor in the Z-up world.
    auto p = plane(3, 3, 2.0, 2.0);
    CHECK(p.triangles.size() == 8);
    int facing_up = 0;
    for (const auto& tri : p.triangles) {
        Vec3 normal = cross(p.vertices[tri[1]] - p.vertices[tri[0]],
                            p.vertices[tri[2]] - p.vertices[tri[0]]);
        if (normal.z > 0.0 && std::abs(normal.x) < 1e-15 && std::abs(normal.y) < 1e-15)
            ++facing_up;
    }
    CHECK(facing_up == 8);
}

static void test_transform() {
    auto c = cube(2.0);

    auto moved = translated(c, Vec3{5.0, -3.0, 2.0});
    CHECK_NEAR(signed_volume(moved), 8.0, 1e-12);
    CHECK_VEC(moved.vertices[0], (Vec3{4.0, -4.0, 1.0}), 1e-15);

    // Non-uniform scale: volume scales by the determinant.
    Mat3 stretch{{{2.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    CHECK_NEAR(signed_volume(transformed(c, stretch, Vec3{0.0, 0.0, 0.0})), 16.0, 1e-12);

    // A reflection has negative determinant: the winding is reversed so the
    // normals stay outward and the volume stays positive.
    Mat3 mirror{{{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    CHECK(mirror.determinant() < 0.0);
    CHECK_NEAR(signed_volume(transformed(c, mirror, Vec3{0.0, 0.0, 0.0})), 8.0, 1e-12);

    // Standing the star up: a quarter turn about x takes the +y tip to +z and
    // the front apex (+z) to -y, toward a camera looking along +y.
    auto up = transformed(star(), rotation_x(pi / 2.0), Vec3{0.0, 0.0, 0.0});
    CHECK_VEC(up.vertices[0], (Vec3{0.0, 0.0, 1.0}), 1e-12);
    CHECK_VEC(up.vertices[10], (Vec3{0.0, -0.3, 0.0}), 1e-12);
    CHECK_NEAR(signed_volume(up), signed_volume(star()), 1e-12);
}

int main() {
    test_closed_primitives();
    test_star();
    test_plane();
    test_transform();
    return check::report("geometry");
}
