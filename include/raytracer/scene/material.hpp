// include/raytracer/scene/material.hpp
#pragma once
#include <cmath>
#include <optional>
#include "raytracer/engine/color.hpp"

namespace raytracer::scene {

    /**
     * How a surface responds to light, following the model of the course
     * notes (2.3 and 2.4). One material belongs to each object of a scene.
     *
     * The specular lobe is tinted with the albedo, as in the notes
     * (rho_s = rho_d): there is no separate specular color.
     */
    struct Material {
        /** Diffuse reflectance per channel, each in [0, 1]. Default: mid gray. */
        raytracer::engine::Color albedo{0.8, 0.8, 0.8};

        /**
         * Exponent s of the specular lobe (cos alpha)^s, larger for a tighter
         * highlight. std::nullopt means a purely matte surface with no lobe;
         * it replaces the sentinel value -1 that a plain number would need.
         */
        std::optional<double> specular_exponent = std::nullopt;

        /** Fraction of the color that comes from the mirror reflection, in [0, 1]. */
        double reflectivity = 0.0;
    };

    /**
     * Checks the ranges the model requires: albedo and reflectivity within
     * [0, 1], and, if present, a finite specular exponent that is not negative.
     * NaN fails every one of these tests.
     * @param material the material to check.
     * @return true if the material is valid.
     */
    inline bool is_valid(const Material& material) {
        auto unit = [](double x) { return x >= 0.0 && x <= 1.0; };
        if (!unit(material.albedo.r) || !unit(material.albedo.g) || !unit(material.albedo.b))
            return false;
        if (!unit(material.reflectivity))
            return false;
        if (material.specular_exponent &&
            !(std::isfinite(*material.specular_exponent) && *material.specular_exponent >= 0.0))
            return false;
        return true;
    }

}  // namespace raytracer::scene
