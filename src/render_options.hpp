// src/render_options.hpp
//
// Command-line options of the raytracer executable. Lives in src/, not in
// include/, because it belongs to the application and not to the libraries.
#pragma once
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include "raytracer/shading/render.hpp"

namespace cli {

    enum class OutputFormat { ppm, png };

    /** One file to write: where, and in which format. */
    struct Output {
        std::string path;
        OutputFormat format;
    };

    namespace detail {

        /**
         * The files an --output value asks for. An extension picks the format
         * (.ppm or .png, in any letter case) and writes that one file. No
         * extension means "both": the path gets .ppm and .png appended, so the
         * raw PPM and a PNG that opens anywhere are kept together.
         * Only the last component of the path counts, so a dot in a directory
         * name (shots/v0.1/final) is not mistaken for an extension.
         */
        inline std::vector<Output> outputs_for(const std::string& path) {
            std::filesystem::path file(path);
            std::string name = file.filename().string();
            if (name.empty() || name == "." || name == "..")
                throw std::invalid_argument("--output: '" + path + "' does not name a file");

            std::string extension = file.extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (extension.empty())
                return {{path + ".ppm", OutputFormat::ppm}, {path + ".png", OutputFormat::png}};
            if (extension == ".ppm")
                return {{path, OutputFormat::ppm}};
            if (extension == ".png")
                return {{path, OutputFormat::png}};
            throw std::invalid_argument("--output: '" + path +
                                        "' must end in .ppm or .png, or have no extension to write both");
        }

    }  // namespace detail

    /** Everything the command line can configure. */
    struct Options {
        std::vector<Output> outputs = detail::outputs_for("gallery");   // gallery.ppm and gallery.png
        int width = 800;
        int height = 300;
        double fov_degrees = 40.0;
        raytracer::shading::RenderSettings settings;
        bool help = false;
    };

    /** @return the help text listing every option and its default. */
    inline std::string usage() {
        return
            "usage: raytracer [options]\n"
            "  --output PATH   .ppm or .png writes only that format; a path with no extension\n"
            "                  writes both PATH.ppm and PATH.png (default gallery)\n"
            "  --width N       image width in pixels, 1 to 16384 (default 800)\n"
            "  --height N      image height in pixels, 1 to 16384 (default 300)\n"
            "  --fov DEGREES   vertical field of view, between 0 and 180 (default 40)\n"
            "  --mode MODE     normals | albedo | lambert | distance | object-id (default normals)\n"
            "  --far X         distance drawn as black in distance mode (default 20)\n"
            "  --help          show this message\n";
    }

    namespace detail {

        inline int parse_int(const std::string& flag, const std::string& text) {
            std::size_t used = 0;
            int value = 0;
            try {
                value = std::stoi(text, &used);
            } catch (const std::exception&) {
                throw std::invalid_argument(flag + ": '" + text + "' is not an integer");
            }
            if (used != text.size())
                throw std::invalid_argument(flag + ": '" + text + "' is not an integer");
            return value;
        }

        inline double parse_double(const std::string& flag, const std::string& text) {
            std::size_t used = 0;
            double value = 0.0;
            try {
                value = std::stod(text, &used);
            } catch (const std::exception&) {
                throw std::invalid_argument(flag + ": '" + text + "' is not a number");
            }
            if (used != text.size() || !std::isfinite(value))
                throw std::invalid_argument(flag + ": '" + text + "' is not a finite number");
            return value;
        }

        inline raytracer::shading::ShadingMode mode_from_name(const std::string& name) {
            using raytracer::shading::ShadingMode;
            if (name == "normals")   return ShadingMode::normals;
            if (name == "albedo")    return ShadingMode::albedo;
            if (name == "lambert")   return ShadingMode::lambert;
            if (name == "distance")  return ShadingMode::distance;
            if (name == "object-id") return ShadingMode::object_id;
            throw std::invalid_argument("--mode: unknown mode '" + name +
                                        "' (use normals, albedo, lambert, distance or object-id)");
        }

    }  // namespace detail

    /**
     * Parses the command-line arguments (without the program name).
     * A flag given twice keeps its last value.
     * @param args the arguments, e.g. {"--mode", "distance", "--output", "d"}.
     * @return the options, defaults filled in for whatever was not given.
     * @throws std::invalid_argument for an unknown flag, a missing or malformed
     *         value, or a value out of range; the message says which.
     */
    inline Options parse_options(const std::vector<std::string>& args) {
        Options options;

        for (std::size_t k = 0; k < args.size(); ++k) {
            const std::string& flag = args[k];
            auto value = [&]() -> const std::string& {
                if (k + 1 >= args.size())
                    throw std::invalid_argument(flag + " requires a value");
                return args[++k];
            };

            if (flag == "--help") {
                options.help = true;
            } else if (flag == "--output") {
                options.outputs = detail::outputs_for(value());
            } else if (flag == "--width") {
                options.width = detail::parse_int(flag, value());
                if (options.width < 1 || options.width > 16384)
                    throw std::invalid_argument("--width must be between 1 and 16384");
            } else if (flag == "--height") {
                options.height = detail::parse_int(flag, value());
                if (options.height < 1 || options.height > 16384)
                    throw std::invalid_argument("--height must be between 1 and 16384");
            } else if (flag == "--fov") {
                options.fov_degrees = detail::parse_double(flag, value());
                if (!(options.fov_degrees > 0.0 && options.fov_degrees < 180.0))
                    throw std::invalid_argument("--fov must be between 0 and 180 degrees");
            } else if (flag == "--mode") {
                options.settings.mode = detail::mode_from_name(value());
            } else if (flag == "--far") {
                options.settings.distance_far = detail::parse_double(flag, value());
                if (!(options.settings.distance_far > 0.0))
                    throw std::invalid_argument("--far must be positive");
            } else {
                throw std::invalid_argument("unknown option '" + flag + "'");
            }
        }
        return options;
    }

}  // namespace cli
