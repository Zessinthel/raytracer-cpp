// tests/check.hpp
//
// Minimal check harness: no external dependencies. Each test executable
// runs its checks, prints one line per failure and a summary, and returns a
// non-zero exit code if anything failed, which is what ctest looks at.
#pragma once
#include <cmath>
#include <cstdio>

namespace check {

    inline int& failures() { static int n = 0; return n; }
    inline int& total()    { static int n = 0; return n; }

    inline void that(bool condition, const char* expression, const char* file, int line) {
        ++total();
        if (!condition) {
            ++failures();
            std::printf("  FAIL %s:%d: %s\n", file, line, expression);
        }
    }

    inline void near_value(double actual, double expected, double tolerance,
                           const char* expression, const char* file, int line) {
        ++total();
        // Written so that NaN fails: any comparison with NaN is false.
        if (!(std::abs(actual - expected) <= tolerance)) {
            ++failures();
            std::printf("  FAIL %s:%d: %s (actual %.12g, expected %.12g, tolerance %g)\n",
                        file, line, expression, actual, expected, tolerance);
        }
    }

    inline int report(const char* suite) {
        std::printf("%s: %d/%d checks passed\n", suite, total() - failures(), total());
        return failures() == 0 ? 0 : 1;
    }

}  // namespace check

#define CHECK(condition) \
    ::check::that((condition), #condition, __FILE__, __LINE__)

#define CHECK_NEAR(actual, expected, tolerance) \
    ::check::near_value((actual), (expected), (tolerance), #actual " vs " #expected, __FILE__, __LINE__)

// Component-wise comparison of anything with x, y, z members (Vec3, Color-like).
#define CHECK_VEC(actual, expected, tolerance)                                                      \
    do {                                                                                            \
        const auto check_a_ = (actual);                                                             \
        const auto check_e_ = (expected);                                                           \
        ::check::near_value(check_a_.x, check_e_.x, (tolerance), #actual ".x", __FILE__, __LINE__); \
        ::check::near_value(check_a_.y, check_e_.y, (tolerance), #actual ".y", __FILE__, __LINE__); \
        ::check::near_value(check_a_.z, check_e_.z, (tolerance), #actual ".z", __FILE__, __LINE__); \
    } while (0)
