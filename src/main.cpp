#include <iostream>
#include <cmath>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/mat3.hpp"

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

    return 0;

}