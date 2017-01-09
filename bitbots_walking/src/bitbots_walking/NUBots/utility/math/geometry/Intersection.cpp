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
 * Copyright 2015 NUBots <nubots@nubots.net>
 */

#include "Intersection.h"

namespace utility {
namespace math {
namespace geometry {
namespace intersection {

bool test(const Circle& circle, const RotatedRectangle& rect) {
	/*
		Let E be the centre of the rectangle (i.e. the origin of its local coordinate frame).
		Define the regions A-I as below, where the central region E has the dimensions of the rectangle.
		
		A │ B │ C
		──┼───┼──
		D │ E │ F
		──┼───┼──
		G │ H │ I

		Check distance to side in regions: B, D, F, and H.
		Check distance to corner in regions: A, C, G, and I.
		Region E is always an intersection.
		Note: This diagram and the circle are symmetric, so we can use absolute values to simplify the comparisons.
		i.e.
          hw
		B │ C
		──┼── hh
		E │ F
	*/

	Transform2D trans = rect.getTransform();
	Transform2D pos = trans.worldToLocal({circle.centre(0), circle.centre(1), 0});

	double hw = 0.5 * rect.getSize()(0);
	double hh = 0.5 * rect.getSize()(1);
	double r = circle.radius;

	double x = std::abs(pos(0));
	double y = std::abs(pos(1));

	if (x < hw && y < hh) { // E
		return true;
	}

	if (x < hw && y > hh) { // B
		return y < hh + r;
	}

	if (x > hw && y < hh) { // F
		return x < hw + r;
	}

	// if (x > hw && y > hh) { // C
		arma::vec2 cornerDiff = { hw - x, hh - y };
		return arma::norm(cornerDiff) < r;
	// }
}

}
}
}
}
