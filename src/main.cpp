#include <iostream>
#include <cmath>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/topology.hpp"

int main(){
    using raytracer::engine::Vec3;
    using raytracer::engine::Mat3;
    using raytracer::engine::identity3;
    using raytracer::engine::rotation_z;
    using raytracer::engine::dot;
    using raytracer::engine::cross;
    using raytracer::engine::length;
    using raytracer::engine::normalized;


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

    return 0;

}