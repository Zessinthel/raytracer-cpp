// src/main.cpp
//
// Orchestrator: builds the gallery scene and renders it to a PPM file.
// All numerical checks live in tests/ and run with ctest.
#include <iostream>
#include <numbers>
#include <string>

#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/polyhedra.hpp"
#include "raytracer/engine/transform.hpp"
#include "raytracer/engine/vec3.hpp"
#include "raytracer/io/ppm_writer.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"

int main(int argc, char** argv) {
    using raytracer::engine::Vec3;
    using raytracer::engine::TriangleMesh;
    using raytracer::engine::rotation_x;
    using raytracer::engine::translated;
    using raytracer::engine::transformed;
    using raytracer::physics::Mesh;
    using raytracer::scene::Camera;
    using raytracer::scene::Scene;

    const std::string output_path = (argc > 1) ? argv[1] : "gallery.ppm";

    // World convention: right-handed, Z up. x runs to the right of the image,
    // y is depth (the camera looks toward +y) and z points up.
    Scene gallery;
    auto add = [&gallery](const TriangleMesh& mesh) {
        gallery.add(Mesh::from_triangle_mesh(mesh));
    };

    add(translated(raytracer::engine::sphere(48, 24, 1.0),        Vec3{-7.0, 0.0, 0.0}));
    add(translated(raytracer::engine::torus(48, 24, 1.0, 0.3),    Vec3{-4.5, 0.0, 0.0}));
    add(translated(raytracer::engine::cylinder(10, 32, 0.8, 2.0), Vec3{-1.5, 0.0, 0.0}));
    add(translated(raytracer::engine::cube(1.6),                  Vec3{ 1.5, 0.0, 0.0}));
    add(translated(raytracer::engine::tetrahedron(1.1),           Vec3{ 4.5, 0.0, 0.0}));

    // star() lies in the xy plane with a tip toward +y. A quarter turn about
    // x stands it up in the xz plane: tip toward +z, front apex toward the
    // camera (-y).
    add(transformed(raytracer::engine::star(),
                    rotation_x(std::numbers::pi / 2.0),
                    Vec3{7.0, 0.0, 0.0}));

    Camera camera(Vec3{0.0, -12.0, 6.0},   // origin: in front of the row, raised
                  Vec3{0.0, 0.0, 0.0},     // look_at
                  Vec3{0.0, 0.0, 1.0},     // up
                  40.0,                    // vertical field of view, degrees
                  800, 300);               // resolution

    raytracer::io::write_ppm(output_path, camera, gallery);
    std::cout << "Image written to " << output_path << "\n";
    return 0;
}
