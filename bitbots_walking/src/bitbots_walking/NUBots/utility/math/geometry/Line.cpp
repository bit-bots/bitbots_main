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

#include "Line.h"

namespace utility {
namespace math {
namespace geometry {

    Line::Line() : normal(arma::fill::zeros), distance(0.0) {
    }

    Line::Line(const arma::vec2& n, const double& d) : normal(n), distance(d) {
    }

    Line::Line(const arma::vec2& a, const arma::vec2& b) : normal(arma::fill::zeros), distance(0.0) {
        setFromPoints(std::forward<const arma::vec2&>(a), std::forward<const arma::vec2&>(b));
    }

    void Line::setFromPoints(const arma::vec2& a, const arma::vec2& b) {
        arma::vec2 l = arma::normalise(a - b);

        normal = arma::vec2({ -l[1], l[0] });
        distance = arma::dot(normal, a);
    }

    double Line::x(const double& y) const {
        return (distance - y * normal[1]) / normal[0];
    }

    double Line::y(const double& x) const {
        return (distance - x * normal[0]) / normal[1];
    }

    double Line::distanceToPoint(const arma::vec2& point) const {
        return arma::dot(point, normal) - distance;
    }

    double Line::tangentialDistanceToPoint(const arma::vec2& x) const {
        return arma::dot(tangent(), x);
    }

    arma::vec2 Line::pointFromTangentialDistance(const double& x) const {
        return normal * distance + tangent() * x;
    }

    bool Line::isHorizontal() const {
        return normal[0] == 0;
    }

    bool Line::isVertical() const {
        return normal[1] == 0;
    }

    arma::vec2 Line::orthogonalProjection(const arma::vec2& x) const {
        return x - (arma::dot(x, normal) - distance) * normal;
    }


    Line Line::getParallelLineThrough(const arma::vec2& x) const {
        Line result;
        result.normal = normal;
        result.distance = arma::dot(x, normal);
        return result;
    }

    arma::vec2 Line::tangent() const {
        return {-normal[1], normal[0]};
    }

    arma::vec2 Line::intersect(const Line& line) const {

        arma::vec2 direction1 = tangent();
        arma::vec2 direction2 = line.tangent();
        arma::vec2 point1 = pointFromTangentialDistance(0);
        arma::vec2 point2 = line.pointFromTangentialDistance(0);

        //Setup linear equations:
        arma::mat Ainverse;
        //Check extended lines intersect at all
        double determinant = - direction1[0] * direction2[1] + direction1[1] * direction2[0];
        if (determinant == 0) {
            throw std::domain_error("Line::intersect - Lines do not intersect (parallel)");
        } else {
            Ainverse << -direction2[1] << direction2[0] << arma::endr
                     << -direction1[1] << direction1[0];
            Ainverse *= 1 / determinant;
        }

        arma::vec/*2*/ tValues = Ainverse * (arma::vec(point2) - arma::vec(point1));  //arma::meat

        return point1 + tValues[0] * direction1;
    }

}
}
}

