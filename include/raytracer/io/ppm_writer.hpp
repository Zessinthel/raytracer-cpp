// include/raytracer/io/ppm_writer.hpp
#pragma once
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include "raytracer/scene/camera.hpp"
#include "raytracer/scene/scene.hpp"
#include "raytracer/shading/shading.hpp"

namespace raytracer::io {

/**
 * Clamps x to [0, 1], then maps it to an integer in [0, 255].
 * Values from shading are not guaranteed to stay within [0,1] in
 * general (e.g. future HDR-like lighting could overshoot), so this
 * clamp is what keeps a stray out-of-range value from wrapping
 * into garbage when narrowed to an 8-bit channel.
 * @param x a color channel value, ideally already in [0, 1].
 * @return an integer in [0, 255].
 */
inline int to_byte(double x) {
    double clamped = std::max(0.0, std::min(1.0, x));
    return static_cast<int>(std::round(clamped * 255.0));
}

/**
 * Renders the scene as seen by the camera and writes the result as
 * a PPM (P3, ASCII) image file.
 *
 * Row order: PPM expects rows top-to-bottom, but Camera's j=0 is
 * the bottom of the viewport (by construction, see Camera::ray_for_pixel) —
 * so this function iterates j from ny-1 down to 0, not 0 up to ny-1.
 * Getting this backwards produces a vertically flipped image that
 * still "looks plausible" until compared against a known reference.
 *
 * @param path output file path (overwritten if it exists).
 * @param cam the camera defining the viewport and resolution.
 * @param world the scene to render.
 * @throws std::runtime_error if the file cannot be opened for writing.
 */
inline void write_ppm(const std::string& path,
                      const raytracer::scene::Camera& cam,
                      const raytracer::scene::Scene& world)
{
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("write_ppm: cannot open '" + path + "' for writing");

    int nx = cam.width();
    int ny = cam.height();

    out << "P3\n" << nx << " " << ny << "\n255\n";

    for (int j = ny - 1; j >= 0; --j) {
        for (int i = 0; i < nx; ++i) {
            auto ray = cam.ray_for_pixel(i, j);
            auto color = raytracer::shading::shade(ray, world);
            out << to_byte(color.r) << " "
                << to_byte(color.g) << " "
                << to_byte(color.b) << "  ";
        }
        out << "\n";
    }
}

}  // namespace raytracer::io