// include/raytracer/engine/color.hpp
#pragma once

namespace raytracer::engine {

    /**
     * An RGB color, each channel nominally in [0, 1]. Kept distinct from
     * Vec3 despite the identical layout: a color is not a point or a
     * direction in R^3, and giving it its own type prevents passing a color
     * where geometry is expected, or the other way around.
     */
    struct Color {
        double r, g, b;
    };

}  // namespace raytracer::engine
