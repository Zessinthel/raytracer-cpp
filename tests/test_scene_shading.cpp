// tests/test_scene_shading.cpp
//
// Checks for the camera (right-handed, Z-up), the scene (closest object) and
// the current shading (background gradient and normal-based color).
// Links against `shading`, `scene`, `physics` and `engine`.
#include <cmath>

#include "check.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/ray.hpp"
#include "raytracer/engine/transform.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/shading.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;
using raytracer::scene::Camera;
using raytracer::scene::Scene;
using raytracer::shading::Color;
using raytracer::shading::background_color;
using raytracer::shading::shade;

static void expect_color(const Color& actual, double r, double g, double b, double tolerance) {
    CHECK_NEAR(actual.r, r, tolerance);
    CHECK_NEAR(actual.g, g, tolerance);
    CHECK_NEAR(actual.b, b, tolerance);
}

static void test_camera_axes() {
    // Camera at the origin looking along +y with z up: the corner rays of a
    // 2x2 image at 90 degrees are (+-0.5, 1, +-0.5) normalized.
    Camera cam(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}, 90.0, 2, 2);
    Ray lower_left = cam.ray_for_pixel(0, 0);
    Ray upper_right = cam.ray_for_pixel(1, 1);
    CHECK_VEC(lower_left.direction, (Vec3{-0.408248290463863, 0.816496580927726, -0.408248290463863}), 1e-12);
    CHECK_VEC(upper_right.direction, (Vec3{0.408248290463863, 0.816496580927726, 0.408248290463863}), 1e-12);
    CHECK_NEAR(length(lower_left.direction), 1.0, 1e-12);   // primary rays are unit vectors

    // Right-handed with z up: +x is to the right of the image, +z is up.
    Camera wide(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}, 60.0, 101, 101);
    CHECK(wide.ray_for_pixel(100, 50).direction.x > 0.0);
    CHECK(wide.ray_for_pixel(0, 50).direction.x < 0.0);
    CHECK(wide.ray_for_pixel(50, 100).direction.z > 0.0);
    CHECK(wide.ray_for_pixel(50, 0).direction.z < 0.0);
}

static void test_scene() {
    // Two spheres in a row along +y; the ray leaves the origin slightly off
    // the axes. The nearer one was added second, so index 1 must win.
    Scene row;
    row.add(Mesh::from_triangle_mesh(translated(sphere(32, 32, 1.0), Vec3{0.0, 10.0, 0.0})));
    row.add(Mesh::from_triangle_mesh(translated(sphere(32, 32, 1.0), Vec3{0.0, 5.0, 0.0})));
    CHECK(row.object_count() == 2);

    Ray ray{Vec3{0.0, 0.0, 0.0}, normalized(Vec3{0.05, 1.0, 0.03})};
    auto hit = row.intersect(ray);
    CHECK(hit.has_value());
    if (hit) {
        CHECK(hit->object_index == 1);
        CHECK(hit->t > 3.9 && hit->t < 4.2);
        CHECK_VEC(hit->point, ray.at(hit->t), 1e-12);
    }
    CHECK(!row.intersect(Ray{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, -1.0, 0.0}}).has_value());

    // The camera and the scene agree on left and right: a sphere at x = -3
    // is seen on the left half of the image and not on the right half.
    Scene left;
    left.add(Mesh::from_triangle_mesh(translated(sphere(32, 32, 1.0), Vec3{-3.0, 6.0, 0.0})));
    Camera cam(Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}, 60.0, 101, 101);
    CHECK(left.intersect(cam.ray_for_pixel(7, 50)).has_value());
    CHECK(!left.intersect(cam.ray_for_pixel(93, 50)).has_value());
}

static void test_shading() {
    Vec3 origin{0.0, 0.0, 0.0};

    // The sky gradient runs along z: white looking straight down, soft blue
    // straight up, halfway on the horizon.
    expect_color(background_color(Ray{origin, Vec3{0.0, 0.0, 1.0}}), 0.5, 0.7, 1.0, 1e-12);
    expect_color(background_color(Ray{origin, Vec3{0.0, 0.0, -1.0}}), 1.0, 1.0, 1.0, 1e-12);
    expect_color(background_color(Ray{origin, Vec3{1.0, 0.0, 0.0}}), 0.75, 0.85, 1.0, 1e-12);
    // The direction is normalized inside, so an unnormalized ray gives the
    // same color (this once produced negative channels).
    expect_color(background_color(Ray{origin, Vec3{0.0, 0.0, 5.0}}), 0.5, 0.7, 1.0, 1e-12);

    // A floor facing +z, hit from above, is colored by its normal (0,0,1):
    // (n + 1) / 2 = (0.5, 0.5, 1).
    Scene floor;
    floor.add(Mesh::from_triangle_mesh(plane(3, 3, 10.0, 10.0)));
    expect_color(shade(Ray{Vec3{0.1, 0.2, 5.0}, Vec3{0.0, 0.0, -1.0}}, floor), 0.5, 0.5, 1.0, 1e-12);
    // Looking away from it, the ray sees the sky.
    expect_color(shade(Ray{Vec3{0.1, 0.2, 5.0}, Vec3{0.0, 0.0, 1.0}}, floor), 0.5, 0.7, 1.0, 1e-12);
}

int main() {
    test_camera_axes();
    test_scene();
    test_shading();
    return check::report("scene_shading");
}
