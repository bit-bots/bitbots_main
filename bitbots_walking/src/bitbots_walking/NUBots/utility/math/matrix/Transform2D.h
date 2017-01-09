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

#ifndef UTILITY_MATH_MATRIX_TRANSFORM2D_H
#define UTILITY_MATH_MATRIX_TRANSFORM2D_H

#include <armadillo>
#include "Rotation2D.h"

namespace utility {
namespace math {
namespace matrix {

    template <int Dimensions>
    class Transform;

    using Transform2D = Transform<2>;

    /**
     * Represents a 2D point including its rotation. Uses a vec3 of the form {x, y, angle}.
     *
     * See: Special Euclidean group SE(2).
     * http://en.wikipedia.org/wiki/Euclidean_group
     *
     * @author Brendan Annable
     */
    template <>
    class Transform<2> : public arma::vec3 {
        using arma::vec3::vec3; // inherit constructors

        public:
            /**
             * @brief Default constructor initialises values to zero
             */
            Transform();

            /**
             * @brief Construct transform from a position and an angle.
             */
            Transform(const arma::vec2 xy_, double angle_);

            /**
             * Construct a transform that represents the position and
             * orientation of a camera positioned at 'from' and facing toward
             * 'to'.
             */
            static Transform2D lookAt(const arma::vec2 from, arma::vec2 to);

            /**
             * @brief Transforms position from local coordinates relative to 'reference', to world coordinates
             *
             * @param reference A position to become relatively local to
             * @return The new position
             */
            Transform2D localToWorld(const Transform2D& reference) const;

            /**
             * @brief Transforms position from world coordinates to be local to 'reference'
             *
             * @param reference The position that the current position is relative to
             * @return The new position
             */
            Transform2D worldToLocal(const Transform2D& reference) const;


            /**
             * Interpolate between itself and given target vector
             *
             * @param t A value between 0-1 to interpolate between the two,
             * outside these bounds will extrapolate
             * @param target The target vector
             * @return The interpolated vector
             */
            Transform2D interpolate(double t, const Transform2D& target) const;

            inline double x() const { return at(0); }
            inline double& x() { return at(0); }

            inline double y() const { return at(1); }
            inline double& y() { return at(1); }

            inline double angle() const { return at(2); }
            inline double& angle() { return at(2); }

            inline Rotation2D rotation() {return Rotation2D::createRotation(angle());}

            inline const arma::subview_col<double> xy() const { return rows(0,1); }
            inline arma::subview_col<double> xy() { return rows(0,1); }
    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_MATRIX_TRANSFORM2D_H
