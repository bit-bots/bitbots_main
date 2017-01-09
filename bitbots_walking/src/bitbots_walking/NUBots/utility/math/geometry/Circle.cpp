/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "Circle.h"

namespace utility {
namespace math {
namespace geometry {

    Circle::Circle() : radius(0.0), radiusSq(0.0), centre(arma::fill::zeros) {
    }


    Circle::Circle(const double& radius, const arma::vec2& centre)
            : radius(radius), radiusSq(radius * radius), centre(centre) {
    }

    Circle::Circle(const arma::vec2& a, const arma::vec2& b, const arma::vec2& c, const double tolerance)
            : radius(0.0), radiusSq(0.0), centre(arma::fill::zeros) {
        setFromPoints(std::forward<const arma::vec2&>(a), std::forward<const arma::vec2&>(b), std::forward<const arma::vec2&>(c),tolerance);
    }

    bool Circle::setFromPoints(const arma::vec2& a, const arma::vec2& b, const arma::vec2& c, const double tolerance) {

        const arma::vec2 ab = a - b;
        const arma::vec2 bc = b - c;
        double det = ((ab[0] * bc[1]) - (bc[0] * ab[1]));

        if (std::abs(det) < tolerance) {
            return false;
        }
        det = 1.0 / det;

        double b_len_sqr = arma::dot(b, b);

        double ab_norm = (arma::dot(a, a) - b_len_sqr) * 0.5;
        double bc_norm = (b_len_sqr - arma::dot(c, c)) * 0.5;


        centre[0] = ((ab_norm * bc[1]) - (bc_norm * ab[1])) * det;
        centre[1] = ((ab[0] * bc_norm) - (bc[0] * ab_norm)) * det;

        radiusSq = arma::accu(arma::square(a - centre));
        radius = std::sqrt(radiusSq);

        return true;
    }

    double Circle::distanceToPoint(const arma::vec2& point) const {
        return arma::norm(point - centre) - radius;
    }

    double Circle::squaresDifference(const arma::vec2& point) const {
        return arma::accu(arma::square(point - centre)) - radiusSq;
    }

    arma::vec2 Circle::orthogonalProjection(const arma::vec2& point) const {
        return arma::normalise(point - centre) * radius + centre;
    }

    arma::vec2 Circle::getEdgePoints(uint y) const {
        auto edgePoints = getEdgePoints(double(y));
        return {
            std::round(edgePoints[0]),
            std::round(edgePoints[1])
        };
    }

    arma::vec2 Circle::getEdgePoints(double y) const {
        // find the min and max x points on the circle for each given y
        // uses the general equation of a circle and solves for x
        double a = y - centre[1];
        double b = std::sqrt(radius * radius - a * a);
        return {
            centre[0] - b,
            centre[0] + b
        };
    }
}
}
}
