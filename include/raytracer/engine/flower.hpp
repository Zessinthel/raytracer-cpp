// include/raytracer/engine/flower.hpp
#pragma once
#include <cmath>
#include <numbers>
#include "raytracer/engine/discretizer.hpp"
#include "raytracer/engine/parametric_surfaces.hpp"
#include "raytracer/engine/topology.hpp"
#include "raytracer/engine/vec3.hpp"

namespace raytracer::engine {

    namespace detail {

        /**
         * Mathematical modulo, a - n*floor(a/n): the result always has the
         * sign of n (here always non-negative for n > 0), unlike
         * std::fmod(a, n), which has the sign of a instead. For a < 0 the
         * two disagree completely, not just at a rounding boundary: for
         * instance floor_mod(-7.2, 2*pi) is about 5.37, while
         * std::fmod(-7.2, 2*pi) is about -0.92. flower()'s petal envelope
         * is evaluated at negative s (its domain starts at s = -2), so using
         * std::fmod directly here would distort the petals nearest the
         * center, not just round differently.
         * @param a the dividend.
         * @param n the divisor (> 0).
         * @return a value in [0, n).
         */
        inline double floor_mod(double a, double n) {
            double r = std::fmod(a, n);
            return (r < 0.0) ? r + n : r;
        }

    }  // namespace detail

    /**
     * A stylized rose/flower surface: an open (non-manifold-closed) sheet
     * parametrized by a radial coordinate r in [0, 1] and a combined
     * sweep/petal-envelope angle s ranging over many full turns
     * (s_max - s_min = 22*pi by default, eleven revolutions), so the petal
     * shape itself slowly changes as the surface winds around, giving the
     * layered, opening-bud appearance of an actual flower rather than a
     * simple surface of revolution.
     *
     * Structure, in terms of theta(s) = 2*exp(-s/(8*pi)):
     *   x(s)   = 1 - (1/2)*((5/4)*(1 - floor_mod(3.6*s, 2*pi)/pi)^2 - 1/4)^2
     *   y(r,s) = 2*r^2*(1.2*r - 1)^2 * sin(theta(s))
     *   rho    = x(s) * r * sin(theta(s) + y*cos(theta(s)))
     *   X = rho * sin(s),  Y = rho * cos(s)
     *   Z = x(s) * r * cos(theta(s) - y*sin(theta(s)))
     *
     * At r = 0, y = 0 and rho = 0 for every s, so the entire r = 0 edge of
     * the grid collapses to the single point (0,0,0) -- the same kind of
     * degenerate ring as a sphere's pole or a cone's apex, handled the same
     * way: each quad touching it splits into one zero-area triangle
     * (removed by physics::Mesh) and one valid triangle reaching the point.
     * At r = 1, y is not zero in general, so the outer edge is a genuine,
     * non-degenerate boundary curve, and it is a boundary in the topological
     * sense too: unlike sphere or torus, this surface does not close up in
     * either r or s (both grid_triangles_open, no wraparound), so it has a
     * real free edge all around its rim and at its two s-extremes.
     *
     * @param n_r radial samples, including both r=0 and r=1 (>= 2).
     * @param n_s samples along s, including both endpoints (>= 2).
     * @param s_min start of the s range (default -2, matching the design
     *        this was ported from).
     * @param s_max end of the s range (default 22*pi: eleven full turns).
     * @return a TriangleMesh with n_r*n_s vertices.
     */
    inline TriangleMesh flower(int n_r, int n_s,
                              double s_min = -2.0, double s_max = 22.0 * std::numbers::pi) {
        auto rs = linspace(0.0, 1.0, n_r);
        auto ss = linspace(s_min, s_max, n_s);

        TriangleMesh mesh;
        mesh.vertices.reserve(static_cast<std::size_t>(n_r) * static_cast<std::size_t>(n_s));

        for (double r : rs) {
            for (double s : ss) {
                double theta = 2.0 * std::exp(-s / (8.0 * std::numbers::pi));
                double envelope = 1.0 - detail::floor_mod(3.6 * s, 2.0 * std::numbers::pi) / std::numbers::pi;
                double x = 1.0 - 0.5 * std::pow((5.0 / 4.0) * envelope * envelope - 0.25, 2.0);
                double y = 2.0 * r * r * std::pow(1.2 * r - 1.0, 2.0) * std::sin(theta);

                double rho = x * (r * std::sin(theta + y * std::cos(theta)));
                double px = rho * std::sin(s);
                double py = rho * std::cos(s);
                double pz = x * (r * std::cos(theta - y * std::sin(theta)));

                mesh.vertices.push_back(Vec3{px, py, pz});
            }
        }

        mesh.triangles = grid_triangles_open(n_r, n_s);
        return mesh;
    }

}  // namespace raytracer::engine
