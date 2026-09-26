// include/raytracer/engine/discretizer.hpp
#pragma once
#include <vector>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/coordinates.hpp"

namespace raytracer::engine {

    /**
     * Uniform linear range of n samples in [min_val, max_val].
     * If endpoint is true (default), max_val is included as the last
     * sample (n-1 equal steps). If false, max_val is excluded — the n
     * samples cover [min_val, max_val) with n equal steps, useful for
     * periodic parameters (e.g. an azimuthal angle over a full circle,
     * where min_val and max_val represent the same point).
     * @param min_val first sample.
     * @param max_val upper bound (included iff endpoint is true).
     * @param n number of samples (>= 1).
     * @param endpoint whether to include max_val (default true).
     * @return a vector of n doubles.
     */
    inline std::vector<double> linspace(double min_val, double max_val, int n,
                                        bool endpoint = true) {
        std::vector<double> result(n);
        if (n == 1) {
            result[0] = min_val;
            return result;
        }
        double denom = endpoint ? (n - 1) : n;
        double step = (max_val - min_val) / denom;
        for (int i = 0; i < n; ++i)
            result[i] = min_val + i * step;
        return result;
    }
    /**
     * Uniform Cartesian grid over [x_min,x_max] x [y_min,y_max] x [z_min,z_max].
     * Row-major indexing: point(i,j,k) = grid[(i*ny + j)*nz + k].
     * @return a flat vector of nx*ny*nz Vec3 points.
     */
    inline std::vector<Vec3> discretize_cartesian(
        int nx, double x_min, double x_max,
        int ny, double y_min, double y_max,
        int nz, double z_min, double z_max)
    {
        auto xs = linspace(x_min, x_max, nx);
        auto ys = linspace(y_min, y_max, ny);
        auto zs = linspace(z_min, z_max, nz);

        std::vector<Vec3> grid;
        grid.reserve(nx * ny * nz);
        for (double x : xs)
            for (double y : ys)
                for (double z : zs)
                    grid.push_back(Vec3{x, y, z});
        return grid;
    }

    /**
     * Uniform spherical grid over [r_min,r_max] x [theta_min,theta_max]
     * x [phi_min,phi_max], converted to Cartesian points via
     * spherical_to_cartesian.
     * @return a flat vector of nr*ntheta*nphi Vec3 points, row-major.
     */
    inline std::vector<Vec3> discretize_spherical(
        int nr, double r_min, double r_max,
        int ntheta, double theta_min, double theta_max,
        int nphi, double phi_min, double phi_max)
    {
        auto rs     = linspace(r_min, r_max, nr);
        auto thetas = linspace(theta_min, theta_max, ntheta);
        auto phis   = linspace(phi_min, phi_max, nphi);

        std::vector<Vec3> grid;
        grid.reserve(nr * ntheta * nphi);
        for (double r : rs)
            for (double theta : thetas)
                for (double phi : phis)
                    grid.push_back(spherical_to_cartesian(r, theta, phi));
        return grid;
    }

}  // namespace raytracer::engine