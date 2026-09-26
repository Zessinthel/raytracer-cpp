// include/raytracer/engine/coordinates.hpp
#pragma once
#include <cmath>
#include "raytracer/engine/vec3.hpp"

namespace raytracer::engine {

/**
 * Converts spherical coordinates (r, theta, phi) to Cartesian (x, y, z).
 * theta is colatitude (polar angle, measured from +z), phi is azimuth.
 *   x = r * sin(theta) * cos(phi)
 *   y = r * sin(theta) * sin(phi)
 *   z = r * cos(theta)
 * @param r radius.
 * @param theta colatitude, in [0, pi].
 * @param phi azimuth, typically in [0, 2*pi).
 * @return a new Vec3 in Cartesian coordinates.
 */
inline Vec3 spherical_to_cartesian(double r, double theta, double phi) {
    double sin_theta = std::sin(theta);
    return Vec3{
        r * sin_theta * std::cos(phi),
        r * sin_theta * std::sin(phi),
        r * std::cos(theta)
    };
}

/**
 * Converts cylindrical coordinates (rho, phi, z) to Cartesian (x, y, z).
 *   x = rho * cos(phi)
 *   y = rho * sin(phi)
 *   z = z
 * @param rho radial distance from the z-axis.
 * @param phi azimuth, typically in [0, 2*pi).
 * @param z height along the z-axis.
 * @return a new Vec3 in Cartesian coordinates.
 */
inline Vec3 cylindrical_to_cartesian(double rho, double phi, double z) {
    return Vec3{rho * std::cos(phi), rho * std::sin(phi), z};
}

}  // namespace raytracer::engine