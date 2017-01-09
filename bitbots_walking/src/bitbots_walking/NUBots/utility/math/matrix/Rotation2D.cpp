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

#include "Rotation2D.h"

namespace utility {
namespace math {
namespace matrix {

    Rotation2D::Rotation() {
        eye(); // identity matrix by default
    }

    Rotation2D Rotation2D::rotate(double radians) const {
        return *this * createRotation(radians);
    }

    Rotation2D Rotation2D::i() const {
        // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
        // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
        return t();
    }

    Rotation2D Rotation2D::createRotation(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation2D rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << c << -s << arma::endr
                 << s <<  c;
        return rotation;
    }

}
}
}