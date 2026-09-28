// include/raytracer/io/ppm_writer.hpp
#pragma once
#include <fstream>
#include <stdexcept>
#include <string>
#include "raytracer/engine/image.hpp"
#include "raytracer/io/color_bytes.hpp"

namespace raytracer::io {

    /**
     * Writes an image as a PPM file (P3, plain text, 8 bits per channel).
     * Rows are written top to bottom, the order the format expects, which is
     * the storage order of engine::Image, so nothing is flipped here.
     *
     * @param path output file path (overwritten if it exists).
     * @param image the picture to write.
     * @throws std::runtime_error if the file cannot be opened or written.
     */
    inline void write_ppm(const std::string& path, const raytracer::engine::Image& image) {
        std::ofstream out(path);
        if (!out)
            throw std::runtime_error("write_ppm: cannot open '" + path + "' for writing");

        out << "P3\n" << image.width << " " << image.height << "\n255\n";
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                const raytracer::engine::Color& c = image.at(x, y);
                out << to_byte(c.r) << " " << to_byte(c.g) << " " << to_byte(c.b) << "  ";
            }
            out << "\n";
        }

        out.flush();
        if (!out)
            throw std::runtime_error("write_ppm: error while writing '" + path + "'");
    }

}  // namespace raytracer::io
