// include/raytracer/io/png_writer.hpp
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "raytracer/engine/image.hpp"
#include "raytracer/io/color_bytes.hpp"

namespace raytracer::io {

    namespace detail {

        inline void append_be32(std::vector<std::uint8_t>& out, std::uint32_t v) {
            out.push_back(static_cast<std::uint8_t>(v >> 24));
            out.push_back(static_cast<std::uint8_t>(v >> 16));
            out.push_back(static_cast<std::uint8_t>(v >> 8));
            out.push_back(static_cast<std::uint8_t>(v));
        }

        // CRC-32 as PNG defines it (polynomial 0xEDB88320), table driven.
        inline std::uint32_t crc32(const std::vector<std::uint8_t>& bytes) {
            static const std::array<std::uint32_t, 256> table = [] {
                std::array<std::uint32_t, 256> t{};
                for (std::uint32_t n = 0; n < 256; ++n) {
                    std::uint32_t c = n;
                    for (int k = 0; k < 8; ++k)
                        c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
                    t[n] = c;
                }
                return t;
            }();
            std::uint32_t crc = 0xFFFFFFFFu;
            for (std::uint8_t b : bytes)
                crc = table[(crc ^ b) & 0xFFu] ^ (crc >> 8);
            return crc ^ 0xFFFFFFFFu;
        }

        // Adler-32 checksum that closes a zlib stream.
        inline std::uint32_t adler32(const std::vector<std::uint8_t>& bytes) {
            std::uint32_t a = 1, b = 0;
            for (std::uint8_t v : bytes) {
                a = (a + v) % 65521u;
                b = (b + a) % 65521u;
            }
            return (b << 16) | a;
        }

        // A zlib stream made only of "stored" (uncompressed) deflate blocks.
        // It is a valid zlib stream any PNG reader accepts, at the price of
        // no compression, and it needs no compression library.
        inline std::vector<std::uint8_t> zlib_stored(const std::vector<std::uint8_t>& raw) {
            std::vector<std::uint8_t> z{0x78, 0x01};   // deflate, 32 KiB window, no compression
            std::size_t pos = 0;
            do {
                std::size_t len = std::min<std::size_t>(65535, raw.size() - pos);
                bool last = (pos + len == raw.size());
                std::uint16_t len16 = static_cast<std::uint16_t>(len);
                std::uint16_t nlen16 = static_cast<std::uint16_t>(~len16);
                z.push_back(last ? 1 : 0);              // BFINAL, BTYPE = 00 (stored)
                z.push_back(static_cast<std::uint8_t>(len16 & 0xFF));
                z.push_back(static_cast<std::uint8_t>(len16 >> 8));
                z.push_back(static_cast<std::uint8_t>(nlen16 & 0xFF));
                z.push_back(static_cast<std::uint8_t>(nlen16 >> 8));
                z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                                  raw.begin() + static_cast<std::ptrdiff_t>(pos + len));
                pos += len;
            } while (pos < raw.size());
            append_be32(z, adler32(raw));
            return z;
        }

        inline void write_chunk(std::ofstream& out, const char (&type)[5], const std::vector<std::uint8_t>& data) {
            std::vector<std::uint8_t> length;
            append_be32(length, static_cast<std::uint32_t>(data.size()));

            std::vector<std::uint8_t> body(type, type + 4);   // the CRC covers type and data
            body.insert(body.end(), data.begin(), data.end());
            std::vector<std::uint8_t> crc;
            append_be32(crc, crc32(body));

            out.write(reinterpret_cast<const char*>(length.data()), 4);
            out.write(reinterpret_cast<const char*>(body.data()), static_cast<std::streamsize>(body.size()));
            out.write(reinterpret_cast<const char*>(crc.data()), 4);
        }

    }  // namespace detail

    /**
     * Writes an image as a PNG file (8-bit RGB, no interlacing, uncompressed).
     * The files are larger than a compressing encoder would produce, about
     * the size of the raw pixels, but the writer is self-contained and the
     * result opens anywhere, GitHub included, which the PPM format does not.
     *
     * @param path output file path (overwritten if it exists).
     * @param image the picture to write.
     * @throws std::runtime_error if the file cannot be opened or written.
     */
    inline void write_png(const std::string& path, const raytracer::engine::Image& image) {
        std::ofstream out(path, std::ios::binary);
        if (!out)
            throw std::runtime_error("write_png: cannot open '" + path + "' for writing");

        const std::uint8_t signature[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
        out.write(reinterpret_cast<const char*>(signature), 8);

        std::vector<std::uint8_t> header;
        detail::append_be32(header, static_cast<std::uint32_t>(image.width));
        detail::append_be32(header, static_cast<std::uint32_t>(image.height));
        header.insert(header.end(), {8, 2, 0, 0, 0});   // 8 bits, RGB, deflate, adaptive filter, no interlace
        detail::write_chunk(out, "IHDR", header);

        // Each scanline is one filter-type byte (0 = none) followed by its RGB bytes.
        std::vector<std::uint8_t> raw;
        raw.reserve(static_cast<std::size_t>(image.height) * (1 + 3 * static_cast<std::size_t>(image.width)));
        for (int y = 0; y < image.height; ++y) {
            raw.push_back(0);
            for (int x = 0; x < image.width; ++x) {
                const raytracer::engine::Color& c = image.at(x, y);
                raw.push_back(static_cast<std::uint8_t>(to_byte(c.r)));
                raw.push_back(static_cast<std::uint8_t>(to_byte(c.g)));
                raw.push_back(static_cast<std::uint8_t>(to_byte(c.b)));
            }
        }
        detail::write_chunk(out, "IDAT", detail::zlib_stored(raw));
        detail::write_chunk(out, "IEND", {});

        out.flush();
        if (!out)
            throw std::runtime_error("write_png: error while writing '" + path + "'");
    }

}  // namespace raytracer::io
