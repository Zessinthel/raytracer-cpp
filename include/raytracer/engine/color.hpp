// include/raytracer/engine/color.hpp
#pragma once
#include <algorithm>

namespace raytracer::engine {

    /**
     * An RGB color, each channel nominally in [0, 1]. Kept distinct from
     * Vec3 despite the identical layout: a color is not a point or a
     * direction in R^3, and giving it its own type prevents passing a color
     * where geometry is expected, or the other way around.
     *
     * Arithmetic on colors may leave [0, 1] (a surface lit by several lights
     * can add up to more than 1). Nothing clamps along the way except
     * saturate(), applied explicitly where the model calls for it (2.4.2's
     * "sat"); the image encoder clamps again regardless, as a last resort.
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

    /** Adds two colors component-wise. */
    inline Color operator+(const Color& a, const Color& b) {
        return Color{a.r + b.r, a.g + b.g, a.b + b.b};
    }

    /**
     * Clamps each channel to [0, 1] (2.4.2's sat(*)): a color assembled from
     * a sum of lights or a mix of a local and a reflected color can leave
     * that range, and this is the point at which the model declares that
     * out-of-range light is simply not representable, rather than letting it
     * propagate unclamped into a further combination (as the recursive
     * reflection color must be clamped at each level before being weighted
     * with the next, per Algorithm 2.9). A NaN channel clamps to 0, the same
     * choice io::to_byte makes for the same reason: a bug upstream should
     * show up as black, not as an arbitrary color.
     * @param c the color to clamp.
     * @return a new Color with every channel in [0, 1].
     */
    inline Color saturate(const Color& c) {
        auto clamp01 = [](double x) {
            if (!(x > 0.0)) return 0.0;   // catches x <= 0 and NaN alike
            return x < 1.0 ? x : 1.0;
        };
        return Color{clamp01(c.r), clamp01(c.g), clamp01(c.b)};
    }

}  // namespace raytracer::engine
