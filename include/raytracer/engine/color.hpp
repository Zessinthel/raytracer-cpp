// include/raytracer/engine/color.hpp
#pragma once

namespace raytracer::engine {

    /**
     * An RGB color, each channel nominally in [0, 1]. Kept distinct from
     * Vec3 despite the identical layout: a color is not a point or a
     * direction in R^3, and giving it its own type prevents passing a color
     * where geometry is expected, or the other way around.
     *
     * Arithmetic on colors may leave [0, 1] (a surface lit by several lights
     * can add up to more than 1). Nothing clamps along the way: the range is
     * enforced once, when the image is encoded to bytes.
     */
    struct Color {
        double r, g, b;
    };

    /** Scales every channel of a color by s. */
    inline Color operator*(const Color& c, double s) {
        return Color{c.r * s, c.g * s, c.b * s};
    }

    /** Scales every channel of a color by s. */
    inline Color operator*(double s, const Color& c) {
        return c * s;
    }

}  // namespace raytracer::engine
