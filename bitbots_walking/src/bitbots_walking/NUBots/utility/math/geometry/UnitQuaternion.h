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

#ifndef UTILITY_MATH_GEOMETRY_UNITQUATERNION_H
#define UTILITY_MATH_GEOMETRY_UNITQUATERNION_H

#include <armadillo>

#include "../matrix/Rotation3D.h"

namespace utility {
namespace math {
namespace matrix {
    template <int Dimensions>
    class Rotation;
    using Rotation3D = Rotation<3>;
}
namespace geometry {

    class UnitQuaternion : public arma::vec4 {
        using arma::vec4::vec4; // inherit constructors

        private:

            /* @brief Constructor for non-unit quaternion for purpose of point representation
            */
            UnitQuaternion(const arma::vec3& v);

        public:
            UnitQuaternion();

            UnitQuaternion(const matrix::Rotation3D& rotation);

            UnitQuaternion operator - (const UnitQuaternion& p) const;

            UnitQuaternion operator * (const UnitQuaternion& p) const;

            UnitQuaternion(double realPart, const arma::vec3& imaginaryPart);

            /*! @brief Creates quaternion which rotates about 3D axis by angle radians
            */
            UnitQuaternion(const arma::vec3& axis, double angle);

            /*! @brief Gets the inverse of the quaternion
            */
            UnitQuaternion i() const;

            arma::vec3 rotateVector(const arma::vec3& v) const;

            arma::vec3 getAxis() const;

            double getAngle() const;

            void setAngle(double angle);

            void scaleAngle(double scale);

            void normalise();

            double norm();

            // real part
            inline double kW() const { return at(0); };
            inline double& kW() { return at(0); };

            inline double kX() const { return at(1); };
            inline double& kX() { return at(1); };

            inline double kY() const { return at(2); };
            inline double& kY() { return at(2); };

            inline double kZ() const { return at(3); };
            inline double& kZ() { return at(3); };

            inline double real() const { return at(0); };
            inline double& real() { return at(0); };

            inline const arma::subview_col<double> imaginary() const { return rows(1,3); }
            inline arma::subview_col<double> imaginary() { return rows(1,3); }

            UnitQuaternion slerp(const UnitQuaternion& p, const double& t);

    };

}
}
}

#endif
