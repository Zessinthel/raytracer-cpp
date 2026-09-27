// include/raytracer/scene/camera.hpp
#pragma once
#include <cmath>
#include <vector>
#include "raytracer/engine/vec3.hpp"
#include "raytracer/engine/mat3.hpp"
#include "raytracer/engine/ray.hpp"

namespace raytracer::scene {

    /**
     * Pinhole camera: builds a viewport from an eye point, a look-at
     * target, an "up" reference, a vertical field of view, and pixel
     * resolution, and produces one Ray per pixel on demand.
     */
    class Camera {
    public:
        /**
         * @param origin the eye point O.
         * @param look_at the point the camera looks toward.
         * @param up reference "up" vector (need not be exactly
         *        perpendicular to the view direction; it is corrected
         *        via cross products below).
         * @param vfov_degrees vertical field of view, in degrees.
         * @param nx, ny pixel resolution.
         */
        Camera(raytracer::engine::Vec3 origin,
            raytracer::engine::Vec3 look_at,
            raytracer::engine::Vec3 up,
            double vfov_degrees,
            int nx, int ny)
            : origin_(origin), nx_(nx), ny_(ny)
        {
            using namespace raytracer::engine;

            double theta = vfov_degrees * M_PI / 180.0;
            double viewport_height = 2.0 * std::tan(theta / 2.0);
            double aspect_ratio = static_cast<double>(nx) / static_cast<double>(ny);
            double viewport_width = aspect_ratio * viewport_height;

            w_ = normalized(origin - look_at);
            u_ = normalized(cross(up, w_));
            v_ = cross(w_, u_);

            horizontal_ = u_ * viewport_width;
            vertical_   = v_ * viewport_height;
            lower_left_ = origin_ - horizontal_ * 0.5 - vertical_ * 0.5 - w_;
        }

        /**
         * The ray through the center of pixel (i, j).
         * Indexing: i in [0, nx), j in [0, ny); j=0 is the bottom row
         * in camera-space (matches the math derivation above) — a PPM
         * writer, which expects top-to-bottom row order, must iterate
         * j from ny-1 down to 0, not 0 up to ny-1.
         * @param i column index.
         * @param j row index.
         * @return a Ray from origin_ through that pixel's center,
         *         normalized direction.
         */
        raytracer::engine::Ray ray_for_pixel(int i, int j) const {
            using namespace raytracer::engine;
            double s = (i + 0.5) / nx_;
            double t = (j + 0.5) / ny_;
            Vec3 point_on_viewport = lower_left_ + horizontal_ * s + vertical_ * t;
            return Ray{origin_, normalized(point_on_viewport - origin_)};
        }

        /**
         * All nx*ny rays, in row-major order: rays[j*nx + i].
         * @return a flat vector of Ray, one per pixel.
         */
        std::vector<raytracer::engine::Ray> generate_rays() const {
            std::vector<raytracer::engine::Ray> rays;
            rays.reserve(nx_ * ny_);
            for (int j = 0; j < ny_; ++j)
                for (int i = 0; i < nx_; ++i)
                    rays.push_back(ray_for_pixel(i, j));
            return rays;
        }

        int width()  const { return nx_; }
        int height() const { return ny_; }

    private:
        raytracer::engine::Vec3 origin_;
        raytracer::engine::Vec3 u_, v_, w_;
        raytracer::engine::Vec3 horizontal_, vertical_, lower_left_;
        int nx_, ny_;
    };

}  // namespace raytracer::scene