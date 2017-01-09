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
#ifndef UTILITY_MATH_GEOMETRY_PARAMETRICLINE_H
#define UTILITY_MATH_GEOMETRY_PARAMETRICLINE_H

#include <armadillo>

namespace utility {
namespace math {
namespace geometry {

    template<int n=2>
    class ParametricLine {
    private:
        using Vector = arma::vec::fixed<n>;

    public:
        Vector direction;
        Vector point;
        arma::vec2 tLimits = {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        ParametricLine() : direction(arma::fill::zeros), point(arma::fill::zeros) {}
        ParametricLine(const Vector& p1, const Vector& p2, bool segment = false) : direction(arma::fill::zeros), point(arma::fill::zeros) {
            setFromTwoPoints(p1, p2, segment);
        };


        arma::vec2 start() const{
            return point + tLimits[0] * direction;
        }
        arma::vec2 end() const{
            return point + tLimits[1] * direction;
        }

        void setFromDirection(const Vector& direction_, const Vector& point_, const arma::vec2& tLimits_ = {-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}){
            if(arma::norm(direction_,1) <= 0){
                throw std::domain_error("ParametricLine::setFromDirection - Direction is zero vector!");
            }
            direction = arma::normalise(direction_);
            point = point_;
            tLimits = tLimits_;
        }

        void setFromTwoPoints(const Vector& p1, const Vector& p2, bool segment = false) {
            double norm = arma::norm(p2 - p1);
            if(norm <= 0){
                throw std::domain_error("ParametricLine::setFromTwoPoints - Two points are identical!");
            }
            direction = (p2 - p1) / norm;
            point = p1;
            if(segment){
                tLimits = arma::vec2({0, norm});
            }
        }

        Vector projectPointToLine(const Vector& p) const {
            Vector x = p - point;
            double tProjection = arma::dot(x,direction);
            return std::min(std::max(tProjection, tLimits[0]),tLimits[1]) * direction + point;
        }

        Vector vectorToLineFromPoint(const Vector& p) const {
            return projectPointToLine(p) - p;
        }

        double distanceToPoint(const Vector& p) const {
            return arma::norm(vectorToLine(p));
        }

        Vector intersect(const ParametricLine<n>& l) const{
            //Do not use for n > 2
            if(n > 2){
                throw std::domain_error("Line::intersect - Lines in more than two dimensions rarely meet! Feature to be added later.");
            }
            //Setup linear equations:
            arma::mat Ainverse;
            //Check extended lines intersect at all
            double determinant = - direction[0] * l.direction[1] + direction[1] * l.direction[0];
            if(determinant == 0){
                throw std::domain_error("Line::intersect - Lines do not intersect (parallel)");
            } else {
                Ainverse << -l.direction[1] << l.direction[0] << arma::endr
                         << -direction[1]   << direction[0];
                Ainverse *= 1 / determinant;
            }

            arma::vec/*2*/ tValues = Ainverse * (arma::vec(l.point) - arma::vec(point));  //arma::meat

            //Check bounds of line segments
            if(tValues[0] < tLimits[0] || tValues[0] > tLimits[1] //ie outside range of first line
            || tValues[1] < l.tLimits[0] || tValues[1] > l.tLimits[1] //outside range of second
              ){
                throw std::domain_error("Line::intersect - Lines do not intersect (tValues out of range)");
            }
            return point + tValues[0] * direction;
        }
    };

}
}
}
#endif
