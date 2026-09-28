// include/raytracer/io/color_bytes.hpp
#pragma once
#include <cmath>

namespace raytracer::io {

    /**
     * Quantizes a color channel to 8 bits: values at or below 0 (and NaN)
     * map to 0, values at or above 1 map to 255, and anything in between is
     * scaled and rounded to the nearest integer (halves round up).
     *
     * The clamp is what keeps an out-of-range value, which an illumination
     * model can produce where several lights add up, from wrapping around
     * into garbage when narrowed to a byte. Mapping NaN to 0 instead of
     * letting it through is deliberate: a NaN channel is a bug upstream and
     * a black pixel is easier to spot than a random one.
     * @param x a color channel value, ideally already in [0, 1].
     * @return an integer in [0, 255].
     */
    inline int to_byte(double x) {
        if (!(x > 0.0))
            return 0;
        if (x >= 1.0)
            return 255;
        return static_cast<int>(std::round(x * 255.0));
    }

}  // namespace raytracer::io
