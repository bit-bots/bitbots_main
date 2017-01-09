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

#include "Polygon.h"
#include "Plane.h"

namespace utility {
namespace math {
namespace geometry {

    Polygon::Polygon(const std::vector<arma::vec2>& vertices) : edges() {
	set(vertices);
    }

    void Polygon::set(const std::vector<arma::vec2>& vertices){
    	for(uint i = 0; i < vertices.size(); i++){
			edges.push_back(ParametricLine<2>());
			edges.back().setFromTwoPoints(vertices[(i+1) % vertices.size()], vertices[i], true);
    	}
    }

    // Use the raycasting method: any plane (equivalent to a line in 2D, but a plane for programming reasons)
    // throught the point will intersect an even number of times with edges of the polygon iff the point lies
    // within the polygon
	bool Polygon::pointContained(const arma::vec2& p) const{
		ParametricLine<2> ray;
		int intersectionCount = 0;
		ray.setFromDirection(arma::vec2{1,0}, p, arma::vec2({0,std::numeric_limits<double>::infinity()}));
		arma::vec2 lastIntersection = {0,0};
		bool hadPreviousIntersection = false;
		for(auto& edge : edges){
			try {
				arma::vec2 intersection = ray.intersect(edge);
				if(hadPreviousIntersection){
					if(arma::norm(intersection - lastIntersection) > 1e-6){
						intersectionCount++;
					}
				} else {
					intersectionCount++;
				}
				hadPreviousIntersection = true;
				lastIntersection = intersection;
			} catch (const std::domain_error& e){
				//We didnt intersect with the line!
				//I know, seems kind of silly to use an exception here, but it works nicely with everything else
			}
		}
		return (intersectionCount % 2) == 1;
	}

	arma::vec2 Polygon::projectPointToPolygon(const arma::vec::fixed<2>& p) const{
		if(pointContained(p)){
			return p;
		}
		double minDistance = std::numeric_limits<double>::infinity();
		arma::vec2 closestPoint;
		for(auto& edge : edges){
			//Get projection
			arma::vec2 proj = edge.projectPointToLine(p);
			double dist = arma::norm(proj - p);
			//If this is closer then update
			if(dist < minDistance){
				minDistance = dist;
				closestPoint = proj;
			}
		}
		return closestPoint;
	}



	// Points should be stored in clockwise order.

}
}
}
