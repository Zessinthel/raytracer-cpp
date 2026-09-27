#include <iostream>
#include <cmath>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/topology.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/physics/triangle.hpp"
#include "raytracer/physics/mesh.hpp"
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"


int main(){
    using raytracer::engine::Vec3;
    using raytracer::engine::Mat3;
    using raytracer::engine::identity3;
    using raytracer::engine::rotation_z;
    using raytracer::engine::dot;
    using raytracer::engine::cross;
    using raytracer::engine::length;
    using raytracer::engine::normalized;
    using raytracer::physics::intersect_triangle;
    using raytracer::engine::sphere;
    using raytracer::engine::Ray;
    using raytracer::physics::Mesh;
    using raytracer::scene::Scene;
    using raytracer::physics::Mesh;

    Vec3 O{0.0, 0.0, -5.0};
    Vec3 C{0.0, 0.0, 0.0};

    double r = 1.0;

    Vec3 L = O-C;
    double alpha0 = dot(L, L) - r * r;
    std::cout << "F(0) = " << alpha0 <<"\n";


    Vec3 D{1.0, 2.0, 2.0};
    double ls = length_squared(D);
    double l  = length(D);
    std::cout << "||D||^2"<< ls << "\n";  
    std::cout << "||D|| = "<< l << "\n";

    Vec3 Dn   = normalized(D);
    std::cout << "D_norm = " << length(Dn) << "\n";

    // --- Vec3: producto cruz, e_x x e_y == e_z ---
    Vec3 ex{1.0, 0.0, 0.0};
    Vec3 ey{0.0, 1.0, 0.0};
    Vec3 ez_calculado = cross(ex, ey);
    std::cout << "e_x x e_y = (" << ez_calculado.x << ", "
               << ez_calculado.y << ", " << ez_calculado.z << ")\n";

    // --- Mat3: identidad no altera el vector ---
    Vec3 X{2.0, 3.0, -1.0};
    Vec3 AX = identity3() * X;
    std::cout << "I*X = (" << AX.x << ", " << AX.y << ", " << AX.z << ")\n";

    // --- Mat3: rotacion de 90 grados en z lleva e_x a e_y ---
    Vec3 Rx = rotation_z(M_PI / 2.0) * ex;
    std::cout << "rotation_z(90deg)*e_x = (" << Rx.x << ", "
               << Rx.y << ", " << Rx.z << ")\n";


    Mat3 I = identity3();
    std::cout << "det(I) = " << I.determinant() << "\n";  // esperas 1

    Mat3 R = rotation_z(M_PI / 4.0);
    Mat3 Rinv = R.inverse();
    Mat3 Rt = R.transpose();
    // esperas Rinv.m[i][j] ≈ Rt.m[i][j] para todo i,j (ortogonalidad)
    std::cout << "R^-1[0][1] = " << Rinv.m[0][1] << ", R^T[0][1] = " << Rt.m[0][1] << "\n";

    auto grid = raytracer::engine::discretize_cartesian(
        2, 0.0, 1.0,
        2, 0.0, 1.0,
        1, 0.0, 0.0
    );
    std::cout << "grid.size() = " << grid.size() << "\n";  // esperas 4 (2*2*1)
    std::cout << "grid[0] = (" << grid[0].x << ", " << grid[0].y << ", " << grid[0].z << ")\n";

    // --- Topology: grid_triangles_open(3,3) debe dar 8 triangulos ---
    using raytracer::engine::grid_triangles_open;

    auto tris = grid_triangles_open(3, 3);
    std::cout << "num_triangulos = " << tris.size() << "\n";  // esperas 8

    for (const auto& t : tris) {
        std::cout << "  {" << t[0] << ", " << t[1] << ", " << t[2] << "}\n";
    }

    // --- ParametricSurfaces: chequeo de conteo para sphere(5,6,1.0) ---
    using raytracer::engine::sphere;

    auto s = sphere(5, 6, 1.0);
    std::cout << "sphere vertices = " << s.vertices.size() << "\n";   // esperas 30
    std::cout << "sphere triangles = " << s.triangles.size() << "\n"; // esperas 44

    // --- Triangle: interseccion Moller-Trumbore ---

    Vec3 v0{-1.0, -1.0, 0.0};
    Vec3 v1{ 1.0, -1.0, 0.0};
    Vec3 v2{ 0.0,  1.0, 0.0};

    // Caso 1: rayo directo al centroide del triangulo, debe impactar
    Vec3 O1{0.0, 0.0, -1.0};
    Vec3 D1{0.0, 0.0,  1.0};
    auto hit1 = intersect_triangle(O1, D1, v0, v1, v2);
    if (hit1) {
        std::cout << "hit1: t=" << hit1->t << " u=" << hit1->u << " v=" << hit1->v << "\n";
    } else {
        std::cout << "hit1: sin interseccion (inesperado)\n";
    }

    // Caso 2: rayo paralelo al plano del triangulo (direccion en el
    // propio plano z=0), debe fallar por det ~= 0
    Vec3 O2{0.0, 0.0, 0.0};
    Vec3 D2{1.0, 0.0, 0.0};
    auto hit2 = intersect_triangle(O2, D2, v0, v1, v2);
    std::cout << "hit2: " << (hit2 ? "interseccion (inesperado)" : "sin interseccion (correcto)") << "\n";

    // Caso 3: rayo que pasa fuera del triangulo (a la derecha, x=5)
    Vec3 O3{5.0, 0.0, -1.0};
    Vec3 D3{0.0, 0.0,  1.0};
    auto hit3 = intersect_triangle(O3, D3, v0, v1, v2);
    std::cout << "hit3: " << (hit3 ? "interseccion (inesperado)" : "sin interseccion (correcto)") << "\n";

    auto raw_sphere = sphere(5, 6, 1.0);
    Mesh mesh = Mesh::from_triangle_mesh(raw_sphere);

    std::cout << "mesh.vertex_count()   = " << mesh.vertex_count() << "\n";   // 30 (sin cambio)
    std::cout << "mesh.triangle_count() = " << mesh.triangle_count() << "\n"; // 48 - 12 = 36

    Ray ray{Vec3{0.0, 0.0, -5.0}, Vec3{0.0, 0.0, 1.0}};
    auto hit = mesh.intersect(ray);
    if (hit) {
        std::cout << "hit: t=" << hit->t << " normal=(" << hit->normal.x
                << ", " << hit->normal.y << ", " << hit->normal.z << ")\n";
    } else {
        std::cout << "hit: sin interseccion (inesperado)\n";
    }

    auto raw_sphere_fine = sphere(64, 64, 1.0);
    Mesh mesh_fine = Mesh::from_triangle_mesh(raw_sphere_fine);
    auto hit_fine = mesh_fine.intersect(ray);
    if (hit_fine) std::cout << "hit_fine: t=" << hit_fine->t << "\n";

    Ray ray_offset{Vec3{0.1, 0.0, -5.0}, Vec3{0.0, 0.0, 1.0}};
    auto hit_offset = mesh_fine.intersect(ray_offset);
    if (hit_offset) std::cout << "hit_offset: t=" << hit_offset->t << "\n";

    // --- Camera: rayo central debe apuntar casi exactamente a (0,0,-1) ---
    using raytracer::scene::Camera;

    Camera cam(
        Vec3{0.0, 0.0, 0.0},   // origin
        Vec3{0.0, 0.0, -1.0},  // look_at
        Vec3{0.0, 1.0, 0.0},   // up
        90.0,                  // vfov_degrees
        2, 2                   // nx, ny
    );

    // pixel central aproximado en una grilla 2x2: cualquiera de los 4
    // esta cerca del centro; tomamos (0,0) y (1,1) para ver ambos extremos
    auto r00 = cam.ray_for_pixel(0, 0);
    auto r11 = cam.ray_for_pixel(1, 1);
    std::cout << "ray(0,0).direction = (" << r00.direction.x << ", "
            << r00.direction.y << ", " << r00.direction.z << ")\n";
    std::cout << "ray(1,1).direction = (" << r11.direction.x << ", "
            << r11.direction.y << ", " << r11.direction.z << ")\n";



    Scene test_scene;
    int idx_far  = test_scene.add(Mesh::from_triangle_mesh(sphere(20, 20, 1.0)));  // en el origen, radio 1
    // la esfera "cercana" la desplazamos manualmente sumando el offset a sus vertices
    auto near_sphere_raw = sphere(20, 20, 1.0);
    for (auto& v : near_sphere_raw.vertices) v = v + Vec3{0.0, 0.0, -3.0};  // centrada en z=-3
    int idx_near = test_scene.add(Mesh::from_triangle_mesh(near_sphere_raw));

    Ray test_ray{Vec3{0.0, 0.0, -10.0}, Vec3{0.0, 0.1, 1.0}};  // offset para evitar el agujero polar
    auto scene_hit = test_scene.intersect(test_ray);
    if (scene_hit) {
        std::cout << "scene_hit: object=" << scene_hit->object_index
                << " t=" << scene_hit->t << "\n";  // esperas object=1 (la cercana), t menor
    } else {
        std::cout << "scene_hit: sin interseccion (inesperado)\n";
    }
    
    return 0;

}