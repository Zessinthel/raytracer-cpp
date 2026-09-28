// tests/test_cli.cpp
//
// Checks for the command-line option parser: defaults, every flag, the output
// format taken from the file extension, and the rejection of anything
// malformed. Links against `shading` (for RenderSettings) and `engine`.
#include <stdexcept>
#include <string>
#include <vector>

#include "check.hpp"
#include "render_options.hpp"

using raytracer::shading::ShadingMode;

static cli::Options parse(std::vector<std::string> args) {
    return cli::parse_options(args);
}

// True if parsing throws std::invalid_argument, false if it succeeds.
static bool rejects(std::vector<std::string> args) {
    try {
        cli::parse_options(args);
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

static void test_defaults() {
    cli::Options o = parse({});
    CHECK(o.output == "gallery.ppm");
    CHECK(o.format == cli::OutputFormat::ppm);
    CHECK(o.width == 800 && o.height == 300);
    CHECK_NEAR(o.fov_degrees, 40.0, 0.0);
    CHECK(o.settings.mode == ShadingMode::normals);
    CHECK_NEAR(o.settings.distance_far, 20.0, 0.0);
    CHECK(!o.help);
}

static void test_flags() {
    cli::Options o = parse({"--width", "1024", "--height", "512", "--fov", "55.5",
                            "--mode", "distance", "--far", "12.5", "--output", "out.png"});
    CHECK(o.width == 1024 && o.height == 512);
    CHECK_NEAR(o.fov_degrees, 55.5, 0.0);
    CHECK(o.settings.mode == ShadingMode::distance);
    CHECK_NEAR(o.settings.distance_far, 12.5, 0.0);
    CHECK(o.output == "out.png");
    CHECK(o.format == cli::OutputFormat::png);

    CHECK(parse({"--mode", "object-id"}).settings.mode == ShadingMode::object_id);
    CHECK(parse({"--mode", "normals"}).settings.mode == ShadingMode::normals);
    CHECK(parse({"--help"}).help);
    CHECK(parse({"--help", "--width", "10"}).help);

    // A repeated flag keeps its last value.
    CHECK(parse({"--width", "100", "--width", "200"}).width == 200);
}

static void test_output_format() {
    CHECK(parse({"--output", "a.ppm"}).format == cli::OutputFormat::ppm);
    CHECK(parse({"--output", "a.png"}).format == cli::OutputFormat::png);
    CHECK(parse({"--output", "DIR/A.PNG"}).format == cli::OutputFormat::png);     // case-insensitive
    CHECK(parse({"--output", "v0.1/shot.final.ppm"}).format == cli::OutputFormat::ppm);
    CHECK(rejects({"--output", "a.jpg"}));
    CHECK(rejects({"--output", "noextension"}));
    CHECK(rejects({"--output", "archive.png.bak"}));
}

static void test_rejections() {
    CHECK(rejects({"--nope"}));                       // unknown flag
    CHECK(rejects({"positional"}));                   // bare argument
    CHECK(rejects({"--width"}));                      // missing value
    CHECK(rejects({"--output"}));
    CHECK(rejects({"--width", "abc"}));               // not a number
    CHECK(rejects({"--width", "12abc"}));             // trailing garbage
    CHECK(rejects({"--width", "3.5"}));               // not an integer
    CHECK(rejects({"--width", "0"}));                 // out of range
    CHECK(rejects({"--width", "-5"}));
    CHECK(rejects({"--width", "16385"}));
    CHECK(!rejects({"--width", "16384"}));
    CHECK(rejects({"--height", "0"}));
    CHECK(rejects({"--fov", "0"}));
    CHECK(rejects({"--fov", "180"}));
    CHECK(rejects({"--fov", "-10"}));
    CHECK(rejects({"--fov", "nan"}));
    CHECK(rejects({"--fov", "inf"}));
    CHECK(rejects({"--far", "0"}));
    CHECK(rejects({"--far", "-1"}));
    CHECK(rejects({"--far", "1e999"}));
    CHECK(rejects({"--mode", "fancy"}));
    CHECK(rejects({"--mode", "Normals"}));            // names are case-sensitive
    CHECK(rejects({"--mode"}));

    // The message names the offending flag, so the user knows what to fix.
    try {
        parse({"--width", "abc"});
    } catch (const std::invalid_argument& error) {
        CHECK(std::string(error.what()).find("--width") != std::string::npos);
    }
}

static void test_usage() {
    std::string text = cli::usage();
    for (const char* flag : {"--output", "--width", "--height", "--fov", "--mode", "--far", "--help"})
        CHECK(text.find(flag) != std::string::npos);
    CHECK(text.find("object-id") != std::string::npos);
}

int main() {
    test_defaults();
    test_flags();
    test_output_format();
    test_rejections();
    test_usage();
    return check::report("cli");
}
