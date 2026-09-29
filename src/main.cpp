// src/main.cpp
//
// Orchestrator: reads the options, builds the gallery scene, renders it and
// writes the image. All numerical checks live in tests/ and run with ctest.
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "raytracer/engine/image.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/transform.hpp"
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
    using raytracer::engine::TriangleMesh;
    using raytracer::engine::Vec3;
    using raytracer::engine::rotation_x;
    using raytracer::engine::transformed;
    using raytracer::engine::translated;
    using raytracer::physics::Mesh;
    using raytracer::scene::Camera;
    using raytracer::scene::DirectionalLight;
    using raytracer::scene::Material;
    using raytracer::scene::PointLight;
    using raytracer::scene::Scene;

    cli::Options options;
    try {
        options = cli::parse_options(std::vector<std::string>(argv + 1, argv + argc));
    } catch (const std::invalid_argument& error) {
        std::cerr << "error: " << error.what() << "\n\n" << cli::usage();
        return 2;
    }
    if (options.help) {
        std::cout << cli::usage();
        return 0;
    }

    // World convention: right-handed, Z up. x runs to the right of the image,
    // y is depth (the camera looks toward +y) and z points up.
    Scene gallery;
    auto add = [&gallery](const TriangleMesh& mesh, const Material& material) {
        gallery.add(Mesh::from_triangle_mesh(mesh), material);
    };

    add(translated(raytracer::engine::sphere(50, 50, 1.0),        Vec3{-4.5, 0.0, -2.0}), Material{.albedo = {0.85, 0.20, 0.20}, .specular_exponent = 500.0, .reflectivity = 0.2});
    add(translated(raytracer::engine::torus(50, 50, 1.0, 0.3),    Vec3{-4.5, 0.0,  2.0}), Material{.albedo = {0.20, 0.45, 0.85}, .reflectivity = 0.3});
    
    add(translated(raytracer::engine::cylinder(50, 50, 0.8, 2.0), Vec3{ 0.0, 0.0, -2.0}), Material{.albedo = {0.25, 0.70, 0.30}, .specular_exponent = 500.0});
    add(translated(raytracer::engine::tetrahedron(1.1),           Vec3{ 0.0, 0.0,  2.0}), Material{.albedo = {0.65, 0.30, 0.70}});

    add(translated(raytracer::engine::cube(1.6),                  Vec3{ 4.5, 0.0, -2.0}), Material{.albedo = {0.95, 0.55, 0.15}, .reflectivity = 0.5}); 


    // star() lies in the xy plane with a tip toward +y. A quarter turn about
    // x stands it up in the xz plane: tip toward +z, front apex toward the
    // camera (-y).
    add(transformed(raytracer::engine::star(),
                    rotation_x(std::numbers::pi / 2.0),
                    Vec3{4.5, 0.0, 2.0}),
        Material{.albedo = {1.00, 0.82, 0.20}, .specular_exponent = 600.0});

    // Lights, with the intensities of the reference scene of the course notes
    // (ambient 0.2, point 0.6, directional 0.2: they add up to 1). Both lights
    // are on the camera's side and above the row, to the left and to the right.
    gallery.set_ambient(0.3);
    gallery.add_light(PointLight{Vec3{-2.0, -8, 4.0}, 0.5});
    //gallery.add_light(PointLight{Vec3{4.0, 0, 3.0}, 0.4});
    gallery.add_light(DirectionalLight{Vec3{0.0, 0.0, 2.0}, 0.2});

    Camera camera(Vec3{0.0, -12.0, 6.0},   // origin: in front of the row, raised
                  Vec3{0.0, 0.0, 0.0},     // look_at
                  Vec3{0.0, 0.0, 1.0},     // up
                  options.fov_degrees,
                  options.width, options.height);

    Image image = raytracer::shading::render(camera, gallery, options.settings);

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
