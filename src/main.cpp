#include <iostream>
#include "raytracer/engine/vec3.hpp"

int main(){
    using raytracer::engine::Vec3;


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
    
    return 0;
}