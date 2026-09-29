// src/flower_main.cpp
//
// A separate scene: just the flower parametric surface (engine::flower),
// nothing else. Kept as its own executable, rather than a --scene flag on
// the main one, so the gallery's own options and defaults are untouched.
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "raytracer/engine/flower.hpp"
#include "raytracer/engine/image.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/io/png_writer.hpp"
#include "raytracer/io/ppm_writer.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/light.hpp"
#include "raytracer/scene/material.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/render.hpp"
#include "render_options.hpp"

int main(int argc, char** argv) {
    using raytracer::engine::Image;
    using raytracer::engine::Vec3;
    using raytracer::engine::flower;
    using raytracer::physics::Mesh;
    using raytracer::scene::Camera;
    using raytracer::scene::DirectionalLight;
    using raytracer::scene::Material;
    using raytracer::scene::PointLight;
    using raytracer::scene::Scene;

    cli::Options options;
    try {
        options = cli::parse_options(std::vector<std::string>(argv + 1, argv + argc), "flower");
    } catch (const std::invalid_argument& error) {
        std::cerr << "error: " << error.what() << "\n\n" << cli::usage("flower_scene", "flower");
        return 2;
    }
    if (options.help) {
        std::cout << cli::usage("flower_scene", "flower");
        return 0;
    }

    // A coarser grid than the one this was ported from (Nr=Ns=1000): the
    // brute-force intersector has no acceleration structure yet, so cost
    // scales with triangle count directly, and this is already tens of
    // thousands of triangles. See engine::flower's own documentation for
    // the shape's structure and the r=0 degenerate ring.
    Scene scene;
    scene.add(Mesh::from_triangle_mesh(flower(60, 260)),
              Material{.albedo = {0.85, 0.20, 0.20}, .specular_exponent = 500.0, .reflectivity = 0.1});

    scene.set_ambient(0.5);
    scene.add_light(PointLight{Vec3{-4, -4, 3.5}, 1.0});
    scene.add_light(PointLight{Vec3{4, -4, 3.5}, 1.0}); 
    scene.add_light(DirectionalLight{Vec3{0.0, 0.0, -4.5}, 0.0});

    // A three-quarter view from above, matching the angle that shows the
    // layered petals rather than looking straight down at the outer rim.
    Camera camera(Vec3{0.0, -4, 4.5}, Vec3{0.0, 0.0, 0.05}, Vec3{0.0, 0.0, 1.0},
                  options.fov_degrees, options.width, options.height);

    Image image = raytracer::shading::render(camera, scene, options.settings);

    try {
        for (const cli::Output& output : options.outputs) {
            if (output.format == cli::OutputFormat::png)
                raytracer::io::write_png(output.path, image);
            else
                raytracer::io::write_ppm(output.path, image);
            std::cout << "Image written to " << output.path << "\n";
        }
    } catch (const std::runtime_error& error) {
        std::cerr << "error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
