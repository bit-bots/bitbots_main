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

#ifndef UTILITY_MATH_GEOMETRY_POLYGON_H
#define UTILITY_MATH_GEOMETRY_POLYGON_H

#include <armadillo>
#include "ParametricLine.h"

namespace utility {
namespace math {
namespace geometry {

    class Polygon {
    private:
    	std::vector<ParametricLine<2>> edges;
    public:
    	Polygon() : edges() {}
    	Polygon(const std::vector<arma::vec2>& vertices);

    	void set(const std::vector<arma::vec2>& vertices);

    	/*! @brief Checks if the point lies within the boundary of the polygon
    	*/
    	bool pointContained(const arma::vec2& p) const;
    	/*! @brief Gets the closest point in the polygon to the specified point
    	*/
    	arma::vec2 projectPointToPolygon(const arma::vec2& p) const;
    };
}
}
}

#endif
