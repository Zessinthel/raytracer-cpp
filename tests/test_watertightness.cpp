// tests/test_watertightness.cpp
//
// Audits every closed primitive in engine for the shared-vertex/edge crack
// documented in physics::intersect_triangle: thousands of rays aimed exactly
// at every vertex and at the midpoint of every shared edge, from random
// distant origins. A ray toward a boundary point of a closed solid must
// register some hit; a total miss is a leak, not merely the wrong triangle.
// Also checks that the bary_eps tolerance this relies on introduces no false
// hit for rays that clearly miss the solid.
// Links against `physics` and `engine`.
#include <array>
#include <cstddef>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "check.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;

namespace {

    // A far point around the mesh's centroid, in a uniformly random direction.
    Vec3 random_far_origin(std::mt19937& rng, const Vec3& centroid, double far) {
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        Vec3 d;
        do {
            d = Vec3{unit(rng), unit(rng), unit(rng)};
        } while (length_squared(d) < 1e-6);
        return centroid + normalized(d) * far;
    }

    struct AuditResult {
        long trials = 0;
        long leaks = 0;
    };

    // Fires trials_per_target rays at every vertex and at the midpoint of
    // every shared edge of raw, from random distant origins, and counts how
    // many produce no hit at all against the constructed Mesh.
    AuditResult audit_leaks(const TriangleMesh& raw, int trials_per_target, unsigned seed) {
        Mesh mesh = Mesh::from_triangle_mesh(raw);

        Vec3 centroid{0.0, 0.0, 0.0};
        for (const Vec3& v : raw.vertices) centroid = centroid + v;
        centroid = centroid * (1.0 / static_cast<double>(raw.vertices.size()));
        double radius = 0.0;
        for (const Vec3& v : raw.vertices) radius = std::max(radius, length(v - centroid));
        double far = radius * 50.0 + 10.0;

        std::mt19937 rng(seed);
        AuditResult result;

        auto fire_at = [&](const Vec3& target) {
            for (int k = 0; k < trials_per_target; ++k) {
                Vec3 origin = random_far_origin(rng, centroid, far);
                Vec3 direction = normalized(target - origin);
                ++result.trials;
                if (!mesh.intersect(Ray{origin, direction}, 0.0, T_INFINITE))
                    ++result.leaks;
            }
        };

        for (const Vec3& v : raw.vertices)
            fire_at(v);

        std::set<std::pair<int, int>> edges;
        for (const auto& tri : raw.triangles)
            for (int k = 0; k < 3; ++k) {
                int a = tri[static_cast<std::size_t>(k)];
                int b = tri[static_cast<std::size_t>((k + 1) % 3)];
                edges.insert({std::min(a, b), std::max(a, b)});
            }
        for (const auto& e : edges)
            fire_at((raw.vertices[static_cast<std::size_t>(e.first)] +
                     raw.vertices[static_cast<std::size_t>(e.second)]) * 0.5);

        return result;
    }

}  // namespace

static void test_no_leaks_at_vertices_or_edges() {
    // Every primitive that is a closed, watertight surface by construction
    // (the open plane is excluded: aiming at one of its edge vertices from a
    // random direction can legitimately miss it, since it is not a solid).
    struct Item {
        const char* name;
        TriangleMesh mesh;
    };
    // Meshes here are deliberately coarse: the crack this audits for occurs
    // at any shared vertex regardless of tessellation fineness, and the
    // brute-force intersector makes cost scale as (vertices+edges) times
    // triangle count, so a fine mesh would make this audit needlessly slow
    // without testing anything a coarse one does not already cover.
    std::vector<Item> items;
    items.push_back({"sphere(12,12)", sphere(12, 12, 1.0)});
    items.push_back({"sphere(24,24)", sphere(24, 24, 1.0)});
    items.push_back({"cylinder(8,16)", cylinder(8, 16, 0.8, 2.0)});
    items.push_back({"cone(8,16)", cone(8, 16, 0.8, 2.0)});
    items.push_back({"torus(24,12)", torus(24, 12, 1.0, 0.3)});
    items.push_back({"cube", cube(2.0)});
    items.push_back({"tetrahedron", tetrahedron(1.0)});
    items.push_back({"star", star()});

    long total_trials = 0, total_leaks = 0;
    for (const auto& item : items) {
        AuditResult r = audit_leaks(item.mesh, 25, 12345);
        CHECK(r.trials > 0);
        CHECK(r.leaks == 0);
        total_trials += r.trials;
        total_leaks += r.leaks;
    }
    CHECK(total_trials > 15000);   // measured: about 22500; the audit is not vacuous
    CHECK(total_leaks == 0);
}

static void test_axial_ray_no_longer_leaks_through_the_pole() {
    // The concrete case found much earlier in this project: a ray exactly
    // along a sphere's axis of symmetry used to slip through the degenerate
    // vertex ring at the pole and hit the far side instead of the near one
    // (t = 6 instead of t = 4, for a unit sphere at the origin viewed from
    // z = -5). This is a permanent regression guard for that specific bug.
    // The axial ray itself only needs a couple of resolutions to confirm the
    // fix generalizes; the fine one stays modest for the same reason as above.
    for (int n : {6, 24}) {
        Mesh ball = Mesh::from_triangle_mesh(sphere(n, n, 1.0));
        auto hit = ball.intersect(Ray{Vec3{0.0, 0.0, -5.0}, Vec3{0.0, 0.0, 1.0}}, 0.0, T_INFINITE);
        CHECK(hit.has_value());
        if (hit) CHECK_NEAR(hit->t, 4.0, 1e-9);
    }
}

static void test_bary_epsilon_introduces_no_false_hit() {
    // Rays whose closest approach to a mesh's centroid clearly exceeds its
    // bounding radius cannot legitimately hit it; bary_eps must not turn any
    // of them into a false positive.
    struct Item {
        const char* name;
        TriangleMesh mesh;
    };
    std::vector<Item> items;
    items.push_back({"sphere", sphere(16, 16, 1.0)});
    items.push_back({"cube", cube(2.0)});
    items.push_back({"tetrahedron", tetrahedron(1.0)});
    items.push_back({"star", star()});

    std::mt19937 rng(999);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    long trials = 0, false_hits = 0;
    for (const auto& item : items) {
        Mesh mesh = Mesh::from_triangle_mesh(item.mesh);
        Vec3 centroid{0.0, 0.0, 0.0};
        for (const Vec3& v : item.mesh.vertices) centroid = centroid + v;
        centroid = centroid * (1.0 / static_cast<double>(item.mesh.vertices.size()));
        double radius = 0.0;
        for (const Vec3& v : item.mesh.vertices) radius = std::max(radius, length(v - centroid));

        for (int k = 0; k < 2000; ++k) {
            Vec3 origin = centroid + Vec3{unit(rng), unit(rng), unit(rng)} * (radius * 20.0);
            Vec3 direction;
            do {
                direction = Vec3{unit(rng), unit(rng), unit(rng)};
            } while (length_squared(direction) < 1e-6);
            direction = normalized(direction);

            double t_closest = dot(centroid - origin, direction);
            Vec3 closest = origin + direction * t_closest;
            if (length(closest - centroid) < radius * 1.5)
                continue;   // might pass close enough to be ambiguous: skip it

            ++trials;
            if (mesh.intersect(Ray{origin, direction}, 0.0, T_INFINITE))
                ++false_hits;
        }
    }
    CHECK(trials > 5000);      // measured: about 7956
    CHECK(false_hits == 0);
}

int main() {
    test_no_leaks_at_vertices_or_edges();
    test_axial_ray_no_longer_leaks_through_the_pole();
    test_bary_epsilon_introduces_no_false_hit();
    return check::report("watertightness");
}
