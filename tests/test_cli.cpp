// tests/test_cli.cpp
//
// Checks for the command-line option parser: defaults, every flag, the files
// an --output value asks for, and the rejection of anything malformed. Links
// against `shading` (for RenderSettings) and `engine`.
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

// True if `outputs` is exactly one file with this path and format.
static bool is_single(const std::vector<cli::Output>& outputs, const std::string& path, cli::OutputFormat format) {
    return outputs.size() == 1 && outputs[0].path == path && outputs[0].format == format;
}

// True if `outputs` is the pair "<stem>.ppm" then "<stem>.png".
static bool is_pair(const std::vector<cli::Output>& outputs, const std::string& stem) {
    return outputs.size() == 2 &&
           outputs[0].path == stem + ".ppm" && outputs[0].format == cli::OutputFormat::ppm &&
           outputs[1].path == stem + ".png" && outputs[1].format == cli::OutputFormat::png;
}

static void test_defaults() {
    cli::Options o = parse({});
    CHECK(is_pair(o.outputs, "gallery"));            // both formats unless told otherwise
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
    CHECK(is_single(o.outputs, "out.png", cli::OutputFormat::png));

    CHECK(parse({"--mode", "object-id"}).settings.mode == ShadingMode::object_id);
    CHECK(parse({"--mode", "albedo"}).settings.mode == ShadingMode::albedo);
    CHECK(parse({"--mode", "lambert"}).settings.mode == ShadingMode::lambert);
    CHECK(parse({"--mode", "phong"}).settings.mode == ShadingMode::phong);
    CHECK(parse({"--mode", "normals"}).settings.mode == ShadingMode::normals);
    CHECK(parse({"--help"}).help);
    CHECK(parse({"--help", "--width", "10"}).help);

    // A repeated flag keeps its last value, --output included: it replaces the
    // earlier request instead of adding to it.
    CHECK(parse({"--width", "100", "--width", "200"}).width == 200);
    CHECK(is_single(parse({"--output", "a.png", "--output", "b.ppm"}).outputs, "b.ppm", cli::OutputFormat::ppm));
}

static void test_outputs() {
    // An extension picks the format and writes only that file.
    CHECK(is_single(parse({"--output", "a.ppm"}).outputs, "a.ppm", cli::OutputFormat::ppm));
    CHECK(is_single(parse({"--output", "a.png"}).outputs, "a.png", cli::OutputFormat::png));
    CHECK(is_single(parse({"--output", "DIR/A.PNG"}).outputs, "DIR/A.PNG", cli::OutputFormat::png));   // case-insensitive
    CHECK(is_single(parse({"--output", "v0.1/shot.final.ppm"}).outputs, "v0.1/shot.final.ppm", cli::OutputFormat::ppm));

    // No extension writes both, raw PPM first, keeping the path as given.
    CHECK(is_pair(parse({"--output", "albedo"}).outputs, "albedo"));
    CHECK(is_pair(parse({"--output", "shots/albedo"}).outputs, "shots/albedo"));
    CHECK(is_pair(parse({"--output", "docs/v0.1/geometry"}).outputs, "docs/v0.1/geometry"));   // a dot in a directory is not an extension
    CHECK(is_pair(parse({"--output", ".hidden"}).outputs, ".hidden"));                          // a leading dot is not an extension either

    // An extension that is neither format is a typo, not a stem.
    CHECK(rejects({"--output", "a.jpg"}));
    CHECK(rejects({"--output", "archive.png.bak"}));
    CHECK(rejects({"--output", "gallery."}));

    // Paths that do not name a file.
    CHECK(rejects({"--output", ""}));
    CHECK(rejects({"--output", "shots/"}));
    CHECK(rejects({"--output", "."}));
    CHECK(rejects({"--output", ".."}));
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
    CHECK(text.find("albedo") != std::string::npos);
    CHECK(text.find("lambert") != std::string::npos);
    CHECK(text.find("phong") != std::string::npos);
    CHECK(text.find("both") != std::string::npos);     // the no-extension rule is documented
}

int main() {
    test_defaults();
    test_flags();
    test_outputs();
    test_rejections();
    test_usage();
    return check::report("cli");
}
