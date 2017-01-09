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

#ifndef UTILITY_MATH_MATRIX_ROTATION2D_H
#define UTILITY_MATH_MATRIX_ROTATION2D_H

#include <armadillo>

namespace utility {
namespace math {
namespace matrix {

    template <int Dimensions>
    class Rotation;

    using Rotation2D = Rotation<2>;

    template <>
    class Rotation<2> : public arma::mat22 {
        using arma::mat22::mat22; // inherit constructors

        public:
            /**
             * @brief Default constructor creates an identity matrix
             */
            Rotation();

            /**
             * @brief Rotates matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation2D rotate(double radians) const;

            /**
             * @brief Performs an inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             *
             * @return The inverse transform
             */
            Rotation2D i() const;

            /**
             * @brief Creates a rotation matrix around the Z axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation2D createRotation(double radians);
    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_MATRIX_ROTATION2D_H
