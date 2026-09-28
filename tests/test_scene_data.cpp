// tests/test_scene_data.cpp
//
// Checks for the data a scene carries besides geometry: materials, lights and
// the ambient light, and the rejection of values outside the ranges the
// illumination model assumes. Links against `scene`, `physics` and `engine`
// only, so it also shows that this data needs nothing from `shading`.
#include <cmath>
#include <limits>
#include <stdexcept>
#include <variant>

#include "check.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/transform.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/material.hpp"
#include "raytracer/scene/scene.hpp"

using namespace raytracer::engine;
using raytracer::physics::Mesh;
using raytracer::scene::DirectionalLight;
using raytracer::scene::Light;
using raytracer::scene::Material;
using raytracer::scene::PointLight;
using raytracer::scene::Scene;
using raytracer::scene::is_valid;

constexpr double nan_value = std::numeric_limits<double>::quiet_NaN();
constexpr double inf_value = std::numeric_limits<double>::infinity();

// True if the callable throws std::invalid_argument.
template <typename F>
static bool rejects(F&& call) {
    try {
        call();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

static Mesh unit_cube() { return Mesh::from_triangle_mesh(cube(1.0)); }

static void test_material_defaults() {
    Material m;
    CHECK_NEAR(m.albedo.r, 0.8, 0.0);
    CHECK_NEAR(m.albedo.g, 0.8, 0.0);
    CHECK_NEAR(m.albedo.b, 0.8, 0.0);
    CHECK(!m.specular_exponent.has_value());     // matte by default
    CHECK_NEAR(m.reflectivity, 0.0, 0.0);
    CHECK(is_valid(m));

    // Named fields keep a definition readable; whatever is not named keeps its default.
    Material shiny{.albedo = {1.0, 0.0, 0.0}, .specular_exponent = 500.0, .reflectivity = 0.2};
    CHECK_NEAR(shiny.albedo.r, 1.0, 0.0);
    CHECK(shiny.specular_exponent.has_value() && *shiny.specular_exponent == 500.0);
    CHECK_NEAR(shiny.reflectivity, 0.2, 0.0);
    CHECK(is_valid(shiny));
}

static void test_material_validation() {
    // The ends of every allowed range are valid.
    CHECK(is_valid(Material{.albedo = {0.0, 0.0, 0.0}}));
    CHECK(is_valid(Material{.albedo = {1.0, 1.0, 1.0}, .reflectivity = 1.0}));
    CHECK(is_valid(Material{.specular_exponent = 0.0}));
    CHECK(is_valid(Material{.specular_exponent = 1.0e6}));

    // Just outside them is not, and neither is NaN.
    CHECK(!is_valid(Material{.albedo = {1.01, 0.5, 0.5}}));
    CHECK(!is_valid(Material{.albedo = {0.5, -0.01, 0.5}}));
    CHECK(!is_valid(Material{.albedo = {0.5, 0.5, nan_value}}));
    CHECK(!is_valid(Material{.reflectivity = 1.5}));
    CHECK(!is_valid(Material{.reflectivity = -0.1}));
    CHECK(!is_valid(Material{.reflectivity = nan_value}));
    CHECK(!is_valid(Material{.specular_exponent = -1.0}));      // the old "no lobe" sentinel is not a value
    CHECK(!is_valid(Material{.specular_exponent = nan_value}));
    CHECK(!is_valid(Material{.specular_exponent = inf_value}));
}

static void test_scene_materials() {
    Scene scene;
    Material red{.albedo = {0.9, 0.1, 0.1}, .specular_exponent = 500.0, .reflectivity = 0.3};
    Material blue{.albedo = {0.1, 0.1, 0.9}};

    CHECK(scene.add(unit_cube(), red) == 0);
    CHECK(scene.add(unit_cube(), blue) == 1);
    CHECK(scene.add(unit_cube()) == 2);            // no material given: the default one
    CHECK(scene.object_count() == 3);

    CHECK_NEAR(scene.material_at(0).albedo.r, 0.9, 0.0);
    CHECK(scene.material_at(0).specular_exponent.has_value() && *scene.material_at(0).specular_exponent == 500.0);
    CHECK_NEAR(scene.material_at(0).reflectivity, 0.3, 0.0);
    CHECK_NEAR(scene.material_at(1).albedo.b, 0.9, 0.0);
    CHECK(!scene.material_at(1).specular_exponent.has_value());
    CHECK_NEAR(scene.material_at(2).albedo.g, 0.8, 0.0);

    // An invalid material is refused and leaves the scene as it was.
    CHECK(rejects([&] { scene.add(unit_cube(), Material{.albedo = {2.0, 0.0, 0.0}}); }));
    CHECK(rejects([&] { scene.add(unit_cube(), Material{.reflectivity = -1.0}); }));
    CHECK(scene.object_count() == 3);

    // The material follows the object, not the order of the query: the hit
    // reports the index, and the index finds the right material.
    Scene row;
    row.add(Mesh::from_triangle_mesh(translated(cube(1.0), Vec3{0.0, 10.0, 0.0})), red);
    row.add(Mesh::from_triangle_mesh(translated(cube(1.0), Vec3{0.0, 5.0, 0.0})), blue);
    auto hit = row.intersect(Ray{Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}}, 0.0, T_INFINITE);
    CHECK(hit.has_value());
    if (hit) {
        CHECK(hit->object_index == 1);
        CHECK_NEAR(row.material_at(static_cast<std::size_t>(hit->object_index)).albedo.b, 0.9, 0.0);
    }
}

static void test_lights() {
    Scene scene;
    CHECK(scene.lights().empty());

    CHECK(scene.add_light(PointLight{Vec3{2.0, 0.0, 1.0}, 0.6}) == 0);
    CHECK(scene.add_light(DirectionalLight{Vec3{1.0, 4.0, 4.0}, 0.2}) == 1);
    CHECK(scene.lights().size() == 2);

    // They come back in order, each as the kind it was added as.
    const auto* point = std::get_if<PointLight>(&scene.lights()[0]);
    const auto* sun = std::get_if<DirectionalLight>(&scene.lights()[1]);
    CHECK(point != nullptr && sun != nullptr);
    if (point && sun) {
        CHECK_VEC(point->position, (Vec3{2.0, 0.0, 1.0}), 0.0);
        CHECK_NEAR(point->intensity, 0.6, 0.0);
        CHECK_VEC(sun->direction_to_light, (Vec3{1.0, 4.0, 4.0}), 0.0);   // stored as given, not normalized
        CHECK_NEAR(sun->intensity, 0.2, 0.0);
    }

    // Zero intensity is a valid, if dark, light. Anything else out of range is refused.
    CHECK(is_valid(Light{PointLight{Vec3{0.0, 0.0, 0.0}, 0.0}}));
    CHECK(!is_valid(Light{PointLight{Vec3{0.0, 0.0, 0.0}, -0.1}}));
    CHECK(!is_valid(Light{PointLight{Vec3{0.0, 0.0, 0.0}, nan_value}}));
    CHECK(!is_valid(Light{PointLight{Vec3{0.0, 0.0, 0.0}, inf_value}}));
    CHECK(!is_valid(Light{PointLight{Vec3{nan_value, 0.0, 0.0}, 0.5}}));
    CHECK(!is_valid(Light{PointLight{Vec3{0.0, inf_value, 0.0}, 0.5}}));
    CHECK(!is_valid(Light{DirectionalLight{Vec3{0.0, 0.0, 0.0}, 0.5}}));        // points nowhere
    CHECK(!is_valid(Light{DirectionalLight{Vec3{nan_value, 1.0, 0.0}, 0.5}}));
    CHECK(!is_valid(Light{DirectionalLight{Vec3{0.0, 0.0, 1.0}, -1.0}}));

    CHECK(rejects([&] { scene.add_light(PointLight{Vec3{0.0, 0.0, 0.0}, -0.5}); }));
    CHECK(rejects([&] { scene.add_light(DirectionalLight{Vec3{0.0, 0.0, 0.0}, 0.5}); }));
    CHECK(scene.lights().size() == 2);            // unchanged by the refusals
}

static void test_ambient() {
    Scene scene;
    CHECK_NEAR(scene.ambient(), 0.0, 0.0);        // no ambient light until it is set

    scene.set_ambient(0.2);
    CHECK_NEAR(scene.ambient(), 0.2, 0.0);
    scene.set_ambient(0.0);
    CHECK_NEAR(scene.ambient(), 0.0, 0.0);

    scene.set_ambient(0.3);
    CHECK(rejects([&] { scene.set_ambient(-0.1); }));
    CHECK(rejects([&] { scene.set_ambient(nan_value); }));
    CHECK(rejects([&] { scene.set_ambient(inf_value); }));
    CHECK_NEAR(scene.ambient(), 0.3, 0.0);        // the previous value survives a refusal
}

int main() {
    test_material_defaults();
    test_material_validation();
    test_scene_materials();
    test_lights();
    test_ambient();
    return check::report("scene_data");
}
