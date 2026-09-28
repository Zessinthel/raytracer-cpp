// include/raytracer/engine/image.hpp
#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>
#include "raytracer/engine/color.hpp"

namespace raytracer::engine {

    /**
     * A raster of colors, stored row-major with row 0 at the TOP of the
     * picture, the order every image file format uses. (Camera pixel indices
     * run the other way, j = 0 at the bottom of the viewport; the renderer
     * does that flip once, when it fills the image.)
     */
    struct Image {
        int width;
        int height;
        std::vector<Color> pixels;

        /**
         * Creates a black image.
         * @param width_ number of columns (> 0).
         * @param height_ number of rows (> 0).
         * @throws std::invalid_argument if either dimension is not positive.
         */
        Image(int width_, int height_)
            : width(width_), height(height_), pixels(checked_count(width_, height_), Color{0.0, 0.0, 0.0}) {}

        /** @return the pixel in column x (0 = left) and row y (0 = top). */
        Color& at(int x, int y) {
            return pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)];
        }

        /** @return the pixel in column x (0 = left) and row y (0 = top). */
        const Color& at(int x, int y) const {
            return pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)];
        }

    private:
        static std::size_t checked_count(int width_, int height_) {
            if (width_ <= 0 || height_ <= 0)
                throw std::invalid_argument("Image: width and height must be positive");
            return static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
        }
    };

}  // namespace raytracer::engine
