// tests/test_lighting.cpp
//
// Checks for the diffuse illumination (ambient light plus Lambert's cosine
// law) and the Phong specular term: exact values on a plane for known angles,
// the rules for point and directional lights, the sum of several lights, the
// light/dark boundary on a sphere against the analytic cosine, the peak and
// measured lobe width of the specular term against the notes' formula, and
// the lambert and phong render modes.
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
using raytracer::shading::phong_color;
using raytracer::shading::phong_light;
using raytracer::shading::shade;
using raytracer::shading::render;

constexpr double pi = std::numbers::pi;
constexpr double degrees = pi / 180.0;

// A point on the plane z = 0 whose surface faces +z; lights are placed around it.
static const Vec3 origin{0.0, 0.0, 0.0};
static const Vec3 up{0.0, 0.0, 1.0};

static void expect_color(const Color& actual, double r, double g, double b, double tolerance) {
    CHECK_NEAR(actual.r, r, tolerance);
    CHECK_NEAR(actual.g, g, tolerance);
    CHECK_NEAR(actual.b, b, tolerance);
}

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

// ---- Phong specular term (2.3.2, Algorithm 2.4 lines 8-12) ----------------

static void test_phong_matches_diffuse_without_exponent() {
    // "Los materiales mate carecen de lobulo especular y el termino se omite
    // para ellos" (2.3.2): phong_color must be the exact same color as
    // diffuse_color for a material with no specular_exponent, at several
    // points and view directions, not just approximately the same. (Calling
    // phong_light directly with some exponent is a different question: it
    // always evaluates the specular branch, so it need not match
    // diffuse_light unless every light's R.V happens to be <= 0.)
    Scene scene;
    scene.set_ambient(0.2);
    scene.add_light(PointLight{Vec3{1.0, -2.0, 3.0}, 0.5});
    scene.add_light(DirectionalLight{Vec3{-1.0, 1.0, 1.0}, 0.3});
    scene.add(Mesh::from_triangle_mesh(plane(3, 3, 10.0, 10.0)), Material{.albedo = {0.6, 0.4, 0.2}});   // no specular_exponent

    for (const Vec3& origin_pt : {Vec3{0.3, -0.2, 5.0}, Vec3{-1.5, 1.0, 5.0}, Vec3{2.0, 2.0, 5.0}}) {
        auto hit = scene.intersect(Ray{origin_pt, Vec3{0.0, 0.0, -1.0}}, 0.0, T_INFINITE);
        CHECK(hit.has_value());
        if (!hit) continue;
        for (const Vec3& view : {Vec3{0.0, 0.0, 1.0}, normalized(Vec3{1.0, -1.0, 2.0}), normalized(Vec3{-2.0, 0.5, 1.0})}) {
            Color a = diffuse_color(scene, *hit);
            Color b = phong_color(scene, *hit, view);
            CHECK_NEAR(a.r, b.r, 0.0);
            CHECK_NEAR(a.g, b.g, 0.0);
            CHECK_NEAR(a.b, b.b, 0.0);
        }
    }
}

// Sets up N = +z, a single directional light straight up (intensity 1, no
// ambient): then N.L = 1 and R = reflect(-L, N) = +z = N, so the mirror
// direction is straight up too, and diffuse contributes exactly 1. This
// isolates the specular term as phong_light(...) - 1 for any view direction
// V = (sin(alpha), 0, cos(alpha)), since R.V = cos(alpha) exactly.
static Scene overhead_light_scene() {
    Scene scene;
    scene.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 1.0});
    return scene;
}

static double specular_at_angle(const Scene& scene, double exponent, double alpha_radians) {
    Vec3 view{std::sin(alpha_radians), 0.0, std::cos(alpha_radians)};
    return phong_light(scene, origin, up, view, exponent) - 1.0;    // subtract the known diffuse term
}

static void test_phong_peak_at_mirror_direction() {
    Scene scene = overhead_light_scene();

    // At alpha = 0 (V = R), the lobe is at its maximum: cos(0)^s = 1.
    CHECK_NEAR(specular_at_angle(scene, 50.0, 0.0), 1.0, 1e-12);

    // Sampling many other directions all around the mirror direction never
    // exceeds the value at the peak, for several exponents.
    for (double s : {5.0, 50.0, 500.0}) {
        double peak = specular_at_angle(scene, s, 0.0);
        int worse = 0;
        for (int k = 1; k <= 40; ++k) {
            double alpha = k * (pi / 2.0) / 40.0;    // 0 to 90 degrees, exclusive of 0
            if (specular_at_angle(scene, s, alpha) > peak + 1e-12)
                ++worse;
        }
        CHECK(worse == 0);
    }

    // Strictly monotonic decrease away from the peak (Algorithm 2.4's cos^s
    // has no other local maximum), checked along a path of increasing angle.
    double previous = specular_at_angle(scene, 50.0, 0.0);
    int increases = 0;
    for (int k = 1; k <= 30; ++k) {
        double value = specular_at_angle(scene, 50.0, k * (pi / 2.0) / 30.0);
        if (value > previous + 1e-12) ++increases;
        previous = value;
    }
    CHECK(increases == 0);

    // Past 90 degrees the light is edge-on to R.V (R.V <= 0): the specular
    // term is exactly 0, not negative, per line 11 of Algorithm 2.4.
    CHECK_NEAR(specular_at_angle(scene, 50.0, pi / 2.0), 0.0, 1e-12);
    CHECK_NEAR(specular_at_angle(scene, 50.0, 2.0 * pi / 3.0), 0.0, 1e-12);
    CHECK_NEAR(specular_at_angle(scene, 50.0, pi), 0.0, 1e-12);
}

static void test_phong_lobe_width_matches_the_notes() {
    // 2.3.2: cos^s(alpha) = exp(s ln cos alpha) ~= exp(-s alpha^2 / 2), a
    // Gaussian of angular width 1/sqrt(s) — the angle at which the exact
    // value is closest to exp(-1/2) ~= 0.6065. The notes round this width to
    // 18 degrees for s = 10, 2.6 degrees for s = 500, 1.8 degrees for
    // s = 1000; checked here directly against the exact formula, not the
    // rounded figures.
    Scene scene = overhead_light_scene();
    double reference = std::exp(-0.5);

    struct Case { double s; double degrees; double tolerance; };
    for (Case c : {Case{10.0, 18.0, 0.5}, Case{500.0, 2.6, 0.05}, Case{1000.0, 1.8, 0.05}}) {
        double predicted_width = 1.0 / std::sqrt(c.s);
        CHECK_NEAR(predicted_width * 180.0 / pi, c.degrees, c.tolerance);

        double exact = specular_at_angle(scene, c.s, predicted_width);
        // The Gaussian is a second-order approximation, so this gap narrows
        // as s grows (alpha shrinks); it is not exact at any finite s.
        double gap_tolerance = 4.0 / c.s;
        CHECK_NEAR(exact, reference, gap_tolerance);
    }

    // The approximation improves monotonically with s (the higher-order terms
    // dropped in the Taylor expansion shrink as alpha = 1/sqrt(s) shrinks).
    double gap_10 = std::abs(specular_at_angle(scene, 10.0, 1.0 / std::sqrt(10.0)) - reference);
    double gap_500 = std::abs(specular_at_angle(scene, 500.0, 1.0 / std::sqrt(500.0)) - reference);
    double gap_1000 = std::abs(specular_at_angle(scene, 1000.0, 1.0 / std::sqrt(1000.0)) - reference);
    CHECK(gap_500 < gap_10);
    CHECK(gap_1000 < gap_500);
}

static void test_phong_restriction_not_redundant_with_cosine() {
    // 2.3.2: "la restriccion [R.V > 0] no es redundante con la parte
    // positiva del coseno de alpha_j ... omitir la restriccion produciria
    // brillos procedentes de fuentes que no iluminan el punto". Build
    // exactly that case: a light just below the horizon (N.L < 0, so line 6
    // of Algorithm 2.4 must discard it outright) whose mirror direction R,
    // if it were computed anyway, would still point toward a grazing viewer
    // (R.V > 0) -- so a check that looked only at R.V would wrongly let this
    // light shine.
    Vec3 light_dir{0.999, 0.0, -0.001};       // barely on the inner side
    CHECK(dot(up, light_dir) < 0.0);          // confirms line 6 would discard it

    Vec3 view{-1.0, 0.0, 0.0};                // a grazing observation
    Vec3 r = reflect(light_dir * -1.0, up);   // what R would be, computed independently of accumulate_light
    CHECK(dot(r, view) > 0.0);                // an R.V-only check would have fired here

    Scene scene;
    scene.add_light(DirectionalLight{light_dir, 1.0});
    double total = phong_light(scene, origin, up, view, 50.0);
    CHECK_NEAR(total, 0.0, 0.0);              // no ambient, and the only light is fully excluded
}

static void test_phong_point_light_direction_recomputed() {
    // The mirror direction depends on L, which for a point light changes
    // from point to point; two points under the same light can have very
    // different specular terms even with the same normal and view.
    Scene scene;
    scene.add_light(PointLight{Vec3{0.0, 0.0, 5.0}, 1.0});

    double under_light = phong_light(scene, Vec3{0.0, 0.0, 0.0}, up, up, 200.0);
    double off_to_the_side = phong_light(scene, Vec3{3.0, 0.0, 0.0}, up, up, 200.0);
    CHECK(under_light > off_to_the_side + 0.1);
}

static void test_phong_mode() {
    RenderSettings phong_settings;
    phong_settings.mode = ShadingMode::phong;

    // A shiny plane (albedo 0.2, s = 200) with an overhead light: looking
    // straight down at the point (0,0,0) from directly above lands exactly on
    // the highlight (V = R = (0,0,1)), where the specular term is exactly 1,
    // so the total illumination is 1 (diffuse) + 1 (specular) = 2 and the
    // color is albedo * 2 = 0.4 -- computed exactly, not just "brighter".
    Scene world;
    world.add(Mesh::from_triangle_mesh(plane(3, 3, 20.0, 20.0)),
              Material{.albedo = {0.2, 0.2, 0.2}, .specular_exponent = 200.0});
    world.add_light(DirectionalLight{Vec3{0.0, 0.0, 1.0}, 1.0});

    Color at_peak = shade(Ray{Vec3{0.0, 0.0, 5.0}, Vec3{0.0, 0.0, -1.0}}, world, phong_settings);
    Color lambert_only = shade(Ray{Vec3{0.0, 0.0, 5.0}, Vec3{0.0, 0.0, -1.0}},
                               world, RenderSettings{.mode = ShadingMode::lambert});
    CHECK_NEAR(lambert_only.r, 0.2, 1e-12);
    CHECK_NEAR(at_peak.r, 0.4, 1e-12);

    // Off to the side, away from the highlight, phong and lambert agree
    // (the specular term has decayed to nothing at s = 200).
    Color at_grazing = shade(Ray{Vec3{8.0, 0.0, 5.0}, normalized(Vec3{-1.0, 0.0, -0.3})}, world, phong_settings);
    Color lambert_grazing = shade(Ray{Vec3{8.0, 0.0, 5.0}, normalized(Vec3{-1.0, 0.0, -0.3})},
                                  world, RenderSettings{.mode = ShadingMode::lambert});
    CHECK_NEAR(at_grazing.r, lambert_grazing.r, 1e-9);

    // A miss still shows the sky, like every picture-like mode. The ray
    // starts above the plane (z = 5) and points further up, so it never
    // meets the z = 0 plane (starting exactly on the plane would self-hit at
    // t = 0, which is a different bug to guard against elsewhere).
    Color sky = shade(Ray{Vec3{0.0, 0.0, 5.0}, Vec3{0.0, 0.0, 1.0}}, world, phong_settings);
    expect_color(sky, 0.5, 0.7, 1.0, 1e-12);
}

int main() {
    test_ambient_only();
    test_directional_cosine();
    test_point_light();
    test_sum_of_lights();
    test_diffuse_color();
    test_sphere_terminator();
    test_lambert_mode();
    test_phong_matches_diffuse_without_exponent();
    test_phong_peak_at_mirror_direction();
    test_phong_lobe_width_matches_the_notes();
    test_phong_restriction_not_redundant_with_cosine();
    test_phong_point_light_direction_recomputed();
    test_phong_mode();
    return check::report("lighting");
}
