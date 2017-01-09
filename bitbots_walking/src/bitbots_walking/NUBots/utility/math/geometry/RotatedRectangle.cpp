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

#include "RotatedRectangle.h"

namespace utility {
namespace math {
namespace geometry {

    RotatedRectangle::RotatedRectangle(const Transform2D& transform_, const arma::vec2& size_)
        : transform(transform_), size(size_) { }

    Transform2D RotatedRectangle::getTransform() const {
        return transform;
    }

    arma::vec2 RotatedRectangle::getPosition()   const {
        return arma::vec(transform.xy());
    }

    double RotatedRectangle::getRotation()       const {
        return transform.angle();
    }

    arma::vec2 RotatedRectangle::getSize()       const {
        return size;
    }

}
}
}
