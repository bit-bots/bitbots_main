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


#ifndef UTILITY_MATH_GEOMETRY_LINE_H
#define UTILITY_MATH_GEOMETRY_LINE_H

#include <armadillo>
#include <iostream>
#include <vector>
#include <limits>

namespace utility {
namespace math {
namespace geometry {

    class Line {
    public:
        arma::vec2 normal;
        double distance;

        Line();

        Line(const arma::vec2& n, const double& d);

        Line(const arma::vec2& a, const arma::vec2& b);

        void setFromPoints(const arma::vec2& a, const arma::vec2& b);

        double x(const double& y) const;
        double y(const double& x) const;

        double distanceToPoint(const arma::vec2& point) const;
        double tangentialDistanceToPoint(const arma::vec2& x) const;
        arma::vec2 pointFromTangentialDistance(const double& x) const;

        bool isHorizontal() const;
        bool isVertical() const;

        arma::vec2 orthogonalProjection(const arma::vec2& x) const;

        Line getParallelLineThrough(const arma::vec2& x) const;

        arma::vec2 tangent() const;
        arma::vec2 intersect(const Line& line) const;

        //Perform a least squares fit on a line, optionally using a distance
        //squared threshold away from the current model to filter candidates
        template <typename Iterator>
        void leastSquaresUpdate(const Iterator& first,
                                const Iterator& last,
                                const double& candidateThreshold = std::numeric_limits<double>::max()) {

            arma::vec2 average({ 0.0, 0.0 });
            arma::mat22 covmat({ 0.0, 0.0, 0.0, 0.0 });
            size_t ctr = 0;
            //step 1: calculate means and grab candidates
            for (auto it = first; it != last; ++it) {
                const double diff = distanceToPoint(*it);
                if ( diff*diff < candidateThreshold ) {
                    average += *it;
                    covmat += *it * (*it).t();
                    ++ctr;
                }
            }

            if (ctr > 1) {
                //normalise the average to save some divides later and make code clearer
                average /= (ctr);
                covmat /= (ctr);

                //step 2: calculate the slope and intercept
                double m = (covmat[1] - average[0] * average[1]);
                if (std::abs(normal[0]) > std::abs(normal[1])) { //check whether to use y=mx+b or x=my+b
                    //make a unit vector at right angles to the direction of slope
                    normal = arma::normalise(
                                arma::vec2({ -1.0, m / (covmat[0] - average[0] * average[0])}));
                } else {
                    //make a unit vector at right angles to the direction of slope
                    normal = arma::normalise(
                                arma::vec2({ 1.0, -m / (covmat[3] - average[1] * average[1]) }));

                }

                //find distance the usual way
                distance = arma::dot(average,normal);
            }
        }
    };

}
}
}
#endif
