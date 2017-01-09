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

#include "Transform2D.h"


#include "utility/math/angle.h"

namespace utility {
namespace math {
namespace matrix {

    using utility::math::angle::normalizeAngle;
    using utility::math::angle::vectorToBearing;

    Transform2D::Transform() {
        zeros();
    }

    Transform2D::Transform(const arma::vec2 xy_, double angle_) {
        xy() = xy_;
        angle() = angle_;
    }

    Transform2D Transform2D::lookAt(const arma::vec2 from, arma::vec2 to) {
        arma::vec2 vecHeading = to - from;
        double angle = vectorToBearing(vecHeading);
        return {from, angle};
    }


    Transform2D Transform2D::localToWorld(const Transform2D& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        // translates to this + rotZ(this.angle) * reference
        return {
            x() + cosAngle * reference.x() - sinAngle * reference.y(),
            y() + sinAngle * reference.x() + cosAngle * reference.y(),
            angle() + reference.angle() // do not use normalizeAngle here, causes bad things when turning! TODO: unsure on cause
        };
    }

    Transform2D Transform2D::worldToLocal(const Transform2D& reference) const {
        double cosAngle = std::cos(angle());
        double sinAngle = std::sin(angle());
        Transform2D diff = reference - *this;
        // translates to rotZ(this.angle) * (reference - this)
        return {
            cosAngle * diff.x() + sinAngle * diff.y(),
            -sinAngle * diff.x() + cosAngle * diff.y(),
            normalizeAngle(diff.angle())
        };
    }

    Transform2D Transform2D::interpolate(double t, const Transform2D& target) const {
        Transform2D result = *this + t * (target - *this);
        result.angle() = normalizeAngle(result.angle());
        return result;
    }

}
}
}
