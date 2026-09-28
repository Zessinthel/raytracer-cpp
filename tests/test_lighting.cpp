// tests/test_lighting.cpp
//
// Checks for the diffuse illumination (ambient light plus Lambert's cosine
// law): exact values on a plane for known angles, the rules for point and
// directional lights, the sum of several lights, the light/dark boundary on a
// sphere against the analytic cosine, and the lambert render mode.
// Links against `shading`, `scene`, `physics` and `engine`.
#include <cmath>
#include <numbers>

#include "check.hpp"
#include "raytracer/engine/color.hpp"
#include "raytracer/engine/image.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/transform.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/material.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/lighting.hpp"
#include "raytracer/shading/render.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;
using raytracer::scene::Camera;
using raytracer::scene::DirectionalLight;
using raytracer::scene::Material;
using raytracer::scene::PointLight;
using raytracer::scene::Scene;
using raytracer::shading::RenderSettings;
using raytracer::shading::ShadingMode;
using raytracer::shading::diffuse_color;
using raytracer::shading::diffuse_light;
using raytracer::shading::shade;

constexpr double pi = std::numbers::pi;
constexpr double degrees = pi / 180.0;

// A point on the plane z = 0 whose surface faces +z; lights are placed around it.
static const Vec3 origin{0.0, 0.0, 0.0};
static const Vec3 up{0.0, 0.0, 1.0};

static void test_ambient_only() {
    Scene scene;
    CHECK_NEAR(diffuse_light(scene, origin, up), 0.0, 0.0);      // no light at all: dark
    scene.set_ambient(0.2);
    CHECK_NEAR(diffuse_light(scene, origin, up), 0.2, 0.0);
    // Ambient light reaches a surface whatever way it faces.
    CHECK_NEAR(diffuse_light(scene, origin, Vec3{0.0, 0.0, -1.0}), 0.2, 0.0);
    CHECK_NEAR(diffuse_light(scene, origin, Vec3{1.0, 0.0, 0.0}), 0.2, 0.0);
}

static void test_directional_cosine() {
    // A plane facing +z, ambient 0.2 and one directional light of intensity 0.6.
    // The light direction makes an angle theta with the normal.
    auto illumination = [](double theta) {
        Scene scene;
        scene.set_ambient(0.2);
        scene.add_light(DirectionalLight{Vec3{std::sin(theta), 0.0, std::cos(theta)}, 0.6});
        return diffuse_light(scene, origin, up);
    };

    CHECK_NEAR(illumination(0.0), 0.2 + 0.6, 1e-15);                    // overhead: I_A + I
    CHECK_NEAR(illumination(30.0 * degrees), 0.2 + 0.6 * std::sqrt(3.0) / 2.0, 1e-15);
    CHECK_NEAR(illumination(60.0 * degrees), 0.2 + 0.3, 1e-15);         // 60 degrees: I_A + I/2
    CHECK_NEAR(illumination(89.0 * degrees), 0.2 + 0.6 * std::cos(89.0 * degrees), 1e-15);
    CHECK_NEAR(illumination(120.0 * degrees), 0.2, 0.0);                // behind the surface: only I_A
    CHECK_NEAR(illumination(180.0 * degrees), 0.2, 0.0);

    // Exactly edge-on contributes nothing.
    Scene edge;
    edge.set_ambient(0.2);
    edge.add_light(DirectionalLight{Vec3{1.0, 0.0, 0.0}, 0.6});
    CHECK_NEAR(diffuse_light(edge, origin, up), 0.2, 0.0);

    // Only the direction of a directional light matters, not the length of its vector.
    Scene unit, scaled;
    unit.add_light(DirectionalLight{Vec3{3.0, 0.0, 4.0}, 1.0});
    scaled.add_light(DirectionalLight{Vec3{30.0, 0.0, 40.0}, 1.0});
    CHECK_NEAR(diffuse_light(unit, origin, up), 0.8, 1e-15);            // cos = 4/5
    CHECK_NEAR(diffuse_light(scaled, origin, up), 0.8, 1e-15);
}

static void test_point_light() {
    auto illumination = [](const Vec3& point, const Vec3& light_position) {
        Scene scene;
        scene.add_light(PointLight{light_position, 1.0});
        return diffuse_light(scene, point, up);
    };

    // Directly above: full strength, and no attenuation with distance (the
    // notes take the factor a_j = 1), so 1, 10 and 1000 units away are the same.
    CHECK_NEAR(illumination(origin, Vec3{0.0, 0.0, 1.0}), 1.0, 1e-15);
    CHECK_NEAR(illumination(origin, Vec3{0.0, 0.0, 10.0}), 1.0, 1e-15);
    CHECK_NEAR(illumination(origin, Vec3{0.0, 0.0, 1000.0}), 1.0, 1e-15);

    // The direction is recomputed at every point: at 45 degrees, and 3-4-5.
    CHECK_NEAR(illumination(origin, Vec3{1.0, 0.0, 1.0}), std::sqrt(0.5), 1e-15);
    CHECK_NEAR(illumination(origin, Vec3{-3.0, 0.0, 4.0}), 0.8, 1e-15);

    // ...relative to the point, not to the origin of the world.
    CHECK_NEAR(illumination(Vec3{2.0, 0.0, 0.0}, Vec3{2.0, 0.0, 3.0}), 1.0, 1e-15);
    CHECK_NEAR(illumination(Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 0.0, 3.0}),
               3.0 / std::sqrt(13.0), 1e-15);

    // Below the surface, or level with it: nothing.
    CHECK_NEAR(illumination(origin, Vec3{0.0, 0.0, -2.0}), 0.0, 0.0);
    CHECK_NEAR(illumination(origin, Vec3{5.0, 0.0, 0.0}), 0.0, 0.0);
    // A light at the point itself has no direction: it contributes nothing and must not produce NaN.
    CHECK_NEAR(illumination(origin, origin), 0.0, 0.0);
}

static void test_sum_of_lights() {
    Scene scene;
    scene.set_ambient(0.2);
    scene.add_light(PointLight{Vec3{1.0, 0.0, 1.0}, 0.6});
    scene.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 0.2});
    CHECK_NEAR(diffuse_light(scene, origin, up), 0.2 + 0.6 * std::sqrt(0.5) + 0.2, 1e-15);

    // A light behind the surface drops out of the sum without affecting the rest.
    scene.add_light(PointLight{Vec3{0.0, 0.0, -5.0}, 0.9});
    scene.add_light(DirectionalLight{Vec3{0.0, 0.0, -1.0}, 0.9});
    CHECK_NEAR(diffuse_light(scene, origin, up), 0.2 + 0.6 * std::sqrt(0.5) + 0.2, 1e-15);

    // Intensities that add up past 1 give an illumination past 1: no clamping here.
    Scene bright;
    bright.set_ambient(0.9);
    bright.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 0.9});
    CHECK_NEAR(diffuse_light(bright, origin, up), 1.8, 1e-15);
}

static void test_diffuse_color() {
    Scene scene;
    scene.add(Mesh::from_triangle_mesh(plane(3, 3, 10.0, 10.0)), Material{.albedo = {0.5, 0.25, 1.0}});
    scene.set_ambient(0.2);
    scene.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 0.6});

    auto hit = scene.intersect(Ray{Vec3{1.0, 1.0, 5.0}, Vec3{0.0, 0.0, -1.0}}, 0.0, T_INFINITE);
    CHECK(hit.has_value());
    if (!hit) return;
    Color c = diffuse_color(scene, *hit);
    CHECK_NEAR(c.r, 0.5 * 0.8, 1e-15);
    CHECK_NEAR(c.g, 0.25 * 0.8, 1e-15);
    CHECK_NEAR(c.b, 1.0 * 0.8, 1e-15);

    // Overexposed light gives a channel above 1; clamping is left to the encoder.
    scene.set_ambient(1.0);
    Color hot = diffuse_color(scene, *hit);
    CHECK_NEAR(hot.b, 1.0 * 1.6, 1e-15);
}

// Shading error of a flat-faceted unit sphere lit by a directional light along
// +x, against the analytic value max(0, P.x) of a smooth sphere. The samples are
// primary rays parallel to y, kept away from the silhouette.
struct TerminatorError {
    int samples = 0;
    int dark_wrong = 0;    // clearly on the lit side (x > 0.1) but shaded dark
    int lit_wrong = 0;     // clearly on the dark side (x < -0.1) but shaded lit
    double worst = 0.0;
    double mean = 0.0;
};

static TerminatorError terminator_error(int resolution) {
    Scene ball;
    ball.add(Mesh::from_triangle_mesh(sphere(resolution, resolution, 1.0)), Material{.albedo = {1.0, 1.0, 1.0}});
    ball.add_light(DirectionalLight{Vec3{1.0, 0.0, 0.0}, 1.0});

    TerminatorError result;
    double sum = 0.0;
    for (int a = -48; a <= 48; ++a) {
        for (int b = -48; b <= 48; ++b) {
            double x0 = a / 50.0, z0 = b / 50.0;
            if (x0 * x0 + z0 * z0 > 0.95)
                continue;
            auto hit = ball.intersect(Ray{Vec3{x0, -5.0, z0}, Vec3{0.0, 1.0, 0.0}}, 0.0, T_INFINITE);
            if (!hit) continue;

            double exact = std::max(0.0, hit->point.x / length(hit->point));
            double mesh = diffuse_light(ball, hit->point, hit->normal);
            double error = std::abs(mesh - exact);
            result.worst = std::max(result.worst, error);
            sum += error;
            ++result.samples;
            if (hit->point.x > 0.1 && mesh <= 0.0) ++result.dark_wrong;
            if (hit->point.x < -0.1 && mesh > 0.0) ++result.lit_wrong;
        }
    }
    result.mean = sum / result.samples;
    return result;
}

static void test_sphere_terminator() {
    // With a smooth sphere the illumination at P would be max(0, P.x): the
    // terminator, the boundary between lit and dark, is the plane x = 0. A mesh
    // has flat facets, so each value is off by the tilt of one facet, and the
    // boundary is exact everywhere except within one facet of x = 0.
    TerminatorError coarse = terminator_error(32);
    TerminatorError fine = terminator_error(64);

    CHECK(fine.samples > 5000);                  // measured: 7441
    CHECK(fine.dark_wrong == 0 && fine.lit_wrong == 0);
    CHECK(coarse.dark_wrong == 0 && coarse.lit_wrong == 0);
    CHECK(fine.worst < 0.07);                    // measured: 0.049
    CHECK(fine.mean < 0.015);                    // measured: 0.009

    // The error is a discretization error, not a bug in the lighting: the
    // tilt of a facet is proportional to the angular size of the grid cell,
    // so doubling the resolution halves the error (measured ratio: 0.50).
    double ratio = fine.worst / coarse.worst;
    CHECK(ratio > 0.40 && ratio < 0.60);
    CHECK(fine.mean / coarse.mean > 0.40 && fine.mean / coarse.mean < 0.60);
}

static void test_lambert_mode() {
    RenderSettings lambert;
    lambert.mode = ShadingMode::lambert;

    // A floor at z = -1 with an overhead light, seen from the origin looking
    // along +y: every floor pixel gets the same color albedo * (I_A + I),
    // because the normal and the light direction are the same everywhere.
    Scene world;
    world.add(Mesh::from_triangle_mesh(translated(plane(2, 2, 400.0, 400.0), Vec3{0.0, 0.0, -1.0})),
              Material{.albedo = {0.5, 0.5, 0.5}});
    world.set_ambient(0.2);
    world.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 0.6});

    Camera cam(origin, Vec3{0.0, 1.0, 0.0}, up, 60.0, 9, 7);
    Image image = render(cam, world, lambert);
    int wrong = 0;
    for (int x = 0; x < 9; ++x) {
        Color floor = image.at(x, 6);                            // bottom row: all floor
        if (std::abs(floor.r - 0.4) > 1e-12 || std::abs(floor.g - 0.4) > 1e-12 || std::abs(floor.b - 0.4) > 1e-12)
            ++wrong;
    }
    CHECK(wrong == 0);
    CHECK(image.at(4, 0).b == 1.0);                              // top row: the sky, untouched by lights

    // No lights and no ambient light: the object is black against the sky.
    Scene dark;
    dark.add(Mesh::from_triangle_mesh(translated(plane(2, 2, 400.0, 400.0), Vec3{0.0, 0.0, -1.0})),
             Material{.albedo = {0.9, 0.9, 0.9}});
    Color black = shade(Ray{origin, normalized(Vec3{0.0, 1.0, -0.5})}, dark, lambert);
    CHECK_NEAR(black.r, 0.0, 0.0);
    CHECK_NEAR(black.g, 0.0, 0.0);
    CHECK_NEAR(black.b, 0.0, 0.0);

    // Ambient light of intensity 1 and no other light shows the albedo itself.
    dark.set_ambient(1.0);
    Color full = shade(Ray{origin, normalized(Vec3{0.0, 1.0, -0.5})}, dark, lambert);
    CHECK_NEAR(full.r, 0.9, 1e-15);
}

int main() {
    test_ambient_only();
    test_directional_cosine();
    test_point_light();
    test_sum_of_lights();
    test_diffuse_color();
    test_sphere_terminator();
    test_lambert_mode();
    return check::report("lighting");
}
