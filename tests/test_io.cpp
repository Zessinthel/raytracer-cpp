// tests/test_io.cpp
//
// Checks for the image writers. The PPM output is compared with the exact
// expected text. The PNG output is decoded by an independent reader written
// here (bitwise CRC-32, Adler-32 by its sum formula, a parser for stored
// deflate blocks), so a slip in the writer's table-driven checksums or block
// layout cannot hide behind the same code. Links against `io` and `engine`.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "check.hpp"
#include "raytracer/engine/image.hpp"
#include "raytracer/io/color_bytes.hpp"
#include "raytracer/io/png_writer.hpp"
#include "raytracer/io/ppm_writer.hpp"

using raytracer::engine::Color;
using raytracer::engine::Image;
using raytracer::io::to_byte;
using raytracer::io::write_png;
using raytracer::io::write_ppm;

static std::vector<std::uint8_t> read_bytes(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

static std::string temp_path(const std::string& name) {
    return (std::filesystem::temp_directory_path() / ("raytracer_test_" + name)).string();
}

// ---- independent PNG reader -------------------------------------------------

static std::uint32_t crc32_bitwise(const std::uint8_t* p, std::size_t n) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= p[i];
        for (int k = 0; k < 8; ++k)
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
    return ~crc;
}

// Adler-32 from its closed form: A = 1 + sum(d_i), B = n + sum((n - i) d_i).
static std::uint32_t adler32_sums(const std::vector<std::uint8_t>& d) {
    std::uint64_t a = 1, b = d.size();
    for (std::size_t i = 0; i < d.size(); ++i) {
        a += d[i];
        b += static_cast<std::uint64_t>(d.size() - i) * d[i];
    }
    return static_cast<std::uint32_t>(((b % 65521u) << 16) | (a % 65521u));
}

static std::uint32_t be32(const std::vector<std::uint8_t>& v, std::size_t pos) {
    return (static_cast<std::uint32_t>(v[pos]) << 24) | (static_cast<std::uint32_t>(v[pos + 1]) << 16) |
           (static_cast<std::uint32_t>(v[pos + 2]) << 8) | static_cast<std::uint32_t>(v[pos + 3]);
}

struct DecodedPng {
    bool ok = false;
    std::string error;
    std::uint32_t width = 0, height = 0;
    std::vector<std::uint8_t> rgb;    // width * height * 3, top row first
    int stored_blocks = 0;
};

static DecodedPng decode_png(const std::vector<std::uint8_t>& file) {
    DecodedPng out;
    auto fail = [&](const std::string& why) { out.error = why; return out; };

    const std::uint8_t signature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
    if (file.size() < 8 || !std::equal(signature, signature + 8, file.begin()))
        return fail("bad signature");

    std::vector<std::uint8_t> idat, header;
    bool saw_end = false;
    std::size_t pos = 8;
    while (pos + 12 <= file.size()) {
        std::uint32_t length = be32(file, pos);
        if (pos + 12 + length > file.size()) return fail("chunk overruns the file");
        std::string type(file.begin() + static_cast<std::ptrdiff_t>(pos + 4), file.begin() + static_cast<std::ptrdiff_t>(pos + 8));
        if (crc32_bitwise(&file[pos + 4], 4 + length) != be32(file, pos + 8 + length))
            return fail("bad CRC in chunk " + type);
        auto data_begin = file.begin() + static_cast<std::ptrdiff_t>(pos + 8);
        if (type == "IHDR") header.assign(data_begin, data_begin + length);
        if (type == "IDAT") idat.insert(idat.end(), data_begin, data_begin + length);
        if (type == "IEND") saw_end = true;
        pos += 12 + length;
    }
    if (!saw_end || pos != file.size()) return fail("missing IEND or trailing bytes");
    if (header.size() != 13) return fail("bad IHDR");
    out.width = be32(header, 0);
    out.height = be32(header, 4);
    if (header[8] != 8 || header[9] != 2 || header[10] != 0 || header[11] != 0 || header[12] != 0)
        return fail("not 8-bit RGB, non-interlaced");

    // zlib: header, stored blocks, Adler-32.
    if (idat.size() < 6 || (idat[0] & 0x0F) != 8 || ((idat[0] << 8) | idat[1]) % 31 != 0)
        return fail("bad zlib header");
    std::vector<std::uint8_t> raw;
    std::size_t z = 2;
    while (true) {
        if (z + 5 > idat.size()) return fail("truncated deflate block");
        std::uint8_t block_header = idat[z++];
        if (((block_header >> 1) & 3) != 0) return fail("block is not stored");
        std::uint16_t len = static_cast<std::uint16_t>(idat[z] | (idat[z + 1] << 8));
        std::uint16_t nlen = static_cast<std::uint16_t>(idat[z + 2] | (idat[z + 3] << 8));
        z += 4;
        if (static_cast<std::uint16_t>(~len) != nlen) return fail("LEN and NLEN disagree");
        if (z + len > idat.size()) return fail("block data overruns");
        raw.insert(raw.end(), idat.begin() + static_cast<std::ptrdiff_t>(z), idat.begin() + static_cast<std::ptrdiff_t>(z + len));
        z += len;
        ++out.stored_blocks;
        if (block_header & 1) break;
    }
    if (z + 4 != idat.size()) return fail("bytes after the last block");
    if (be32(idat, z) != adler32_sums(raw)) return fail("bad Adler-32");

    // Unfilter: every scanline must use filter 0 (none).
    std::size_t stride = 1 + 3 * static_cast<std::size_t>(out.width);
    if (raw.size() != stride * out.height) return fail("wrong amount of pixel data");
    for (std::uint32_t y = 0; y < out.height; ++y) {
        if (raw[y * stride] != 0) return fail("unexpected filter type");
        out.rgb.insert(out.rgb.end(), raw.begin() + static_cast<std::ptrdiff_t>(y * stride + 1),
                                       raw.begin() + static_cast<std::ptrdiff_t>((y + 1) * stride));
    }
    out.ok = true;
    return out;
}

// ---- tests ------------------------------------------------------------------

static void test_to_byte() {
    CHECK(to_byte(0.0) == 0);
    CHECK(to_byte(1.0) == 255);
    CHECK(to_byte(0.5) == 128);                       // 127.5 rounds up
    CHECK(to_byte(0.25) == 64);
    CHECK(to_byte(1.5) == 255);                       // clamped, not wrapped
    CHECK(to_byte(-0.2) == 0);
    CHECK(to_byte(std::numeric_limits<double>::quiet_NaN()) == 0);
    CHECK(to_byte(std::numeric_limits<double>::infinity()) == 255);
    CHECK(to_byte(-std::numeric_limits<double>::infinity()) == 0);
}

static Image sample_image() {
    Image image(3, 2);
    image.at(0, 0) = Color{0.0, 0.0, 0.0};
    image.at(1, 0) = Color{1.0, 1.0, 1.0};
    image.at(2, 0) = Color{0.5, 0.25, 0.75};
    image.at(0, 1) = Color{1.5, -0.2, std::numeric_limits<double>::quiet_NaN()};
    image.at(1, 1) = Color{0.2, 0.4, 0.6};
    image.at(2, 1) = Color{1.0, 0.0, 0.0};
    return image;
}

static void test_ppm() {
    std::string path = temp_path("sample.ppm");
    write_ppm(path, sample_image());

    std::ifstream in(path);
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::remove(path.c_str());

    CHECK(text == "P3\n3 2\n255\n"
                  "0 0 0  255 255 255  128 64 191  \n"
                  "255 0 0  51 102 153  255 0 0  \n");

    bool threw = false;
    try { write_ppm("/nonexistent_directory_for_raytracer/x.ppm", sample_image()); }
    catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
}

static void test_png_small() {
    std::string path = temp_path("sample.png");
    write_png(path, sample_image());
    auto bytes = read_bytes(path);
    std::remove(path.c_str());

    DecodedPng png = decode_png(bytes);
    if (!png.ok) std::printf("  decode error: %s\n", png.error.c_str());
    CHECK(png.ok);
    if (!png.ok) return;

    CHECK(png.width == 3 && png.height == 2);
    const std::uint8_t expected[] = {0, 0, 0,  255, 255, 255,  128, 64, 191,
                                     255, 0, 0,  51, 102, 153,  255, 0, 0};
    CHECK(png.rgb.size() == sizeof(expected));
    CHECK(png.rgb == std::vector<std::uint8_t>(expected, expected + sizeof(expected)));
    CHECK(png.stored_blocks == 1);

    // The IEND chunk is empty, so its CRC is a constant every PNG file ends with.
    const std::uint8_t iend[12] = {0, 0, 0, 0, 'I', 'E', 'N', 'D', 0xAE, 0x42, 0x60, 0x82};
    CHECK(bytes.size() >= 12 && std::equal(iend, iend + 12, bytes.end() - 12));
}

static void test_png_multiple_blocks() {
    // 200 x 120 x 3 bytes plus filter bytes is 72120 bytes, more than the
    // 65535 a stored block can hold, so the stream must span two blocks.
    const int w = 200, h = 120;
    Image image(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            image.at(x, y) = Color{x / 199.0, y / 119.0, 0.5};

    std::string path = temp_path("gradient.png");
    write_png(path, image);
    auto bytes = read_bytes(path);
    std::remove(path.c_str());

    DecodedPng png = decode_png(bytes);
    if (!png.ok) std::printf("  decode error: %s\n", png.error.c_str());
    CHECK(png.ok);
    if (!png.ok) return;

    CHECK(png.width == static_cast<std::uint32_t>(w) && png.height == static_cast<std::uint32_t>(h));
    CHECK(png.stored_blocks == 2);

    int wrong = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            std::size_t i = (static_cast<std::size_t>(y) * w + x) * 3;
            if (png.rgb[i] != std::lround(255.0 * (x / 199.0)) ||
                png.rgb[i + 1] != std::lround(255.0 * (y / 119.0)) ||
                png.rgb[i + 2] != 128)
                ++wrong;
        }
    CHECK(wrong == 0);
}

int main() {
    test_to_byte();
    test_ppm();
    test_png_small();
    test_png_multiple_blocks();
    return check::report("io");
}
