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

#ifndef UTILITY_MATH_MATRIX_ROTATION3D_H
#define UTILITY_MATH_MATRIX_ROTATION3D_H

#include <armadillo>

#include "utility/math/geometry/UnitQuaternion.h"

namespace utility {
namespace math {
namespace geometry {
    class UnitQuaternion;
}
namespace matrix {

    template <int Dimensions>
    class Transform;

    using Transform3D = Transform<3>;

    template <int Dimensions>
    class Rotation;

    using Rotation3D = Rotation<3>;
    using Axis = arma::vec3;
    using AxisAngle = std::pair<Axis, double>;

    template <>
    class Rotation<3> : public arma::mat33 {
        using arma::mat33::mat33; // inherit constructors
        public:
            /**
             * @brief Default constructor creates an identity matrix
             */
            Rotation();

            /**
             * @brief Convert from a quaternions vec4
             */
            Rotation(const geometry::UnitQuaternion& q);

            /**
             * @brief Construct an ONB using a vec3 as the X axis
             */
            Rotation(const arma::vec3& axis);

            Rotation(const Transform3D&) = delete;

            /**
             * @brief Create a rotation matrix based on a vec3 as the X axis and an angle
             */
            Rotation(const arma::vec3& axis, double angle);




            /**
             * @brief Rotates matrix around the local X axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateX(double radians) const;

            /**
             * @brief Rotates matrix around the local Y axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateY(double radians) const;

            /**
             * @brief Rotates matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            Rotation3D rotateZ(double radians) const;

            /**
             * @brief Transforms current rotation from world coordinates (i.e. standard basis) to be local to 'reference'
             *
             * @param reference A rotation matrix to become relatively local to
             * @return The transformed rotation matrix
             */
            Rotation3D worldToLocal(const Rotation3D& reference) const;

            /**
             * @brief Rotations current rotation from local coordinates relative to 'reference', to world coordinates (i.e. standard rotation)
             *
             * @param reference The rotation matrix that the current rotation is relative to
             * @return The transformed rotation matrix
             */
            Rotation3D localToWorld(const Rotation3D& reference) const;

            /**
             * @brief Performs an inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             *
             * @return The inverse transform
             */
            Rotation3D i() const;

            /**
             * @return Pair containing the axis of the rotation as a unit vector followed by the rotation angle.
             */
            AxisAngle axisAngle() const;

            /**
             * @return Retrieve the euler angles (xyz) from the matrix
             */
            arma::vec3 eulerAngles() const;

            /**
             * @return The roll (x-axis) of the rotation matrix
             */
            inline double roll() const { return eulerAngles()[0]; }

            /**
             * @return The pitch (y-axis) of the rotation matrix
             */
            inline double pitch() const { return eulerAngles()[1]; }

            /**
             * @return The yaw (z-axis) of the rotation matrix
             */
            inline double yaw() const { return eulerAngles()[2]; }

            /**
             * @brief Creates a rotation matrix around the X axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationX(double radians);
            static Rotation3D createRotationXJacobian(double radians);

            /**
             * @brief Creates a rotation matrix around the Y axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationY(double radians);
            static Rotation3D createRotationYJacobian(double radians);

            /**
             * @brief Creates a rotation matrix around the Z axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation matrix
             */
            static Rotation3D createRotationZ(double radians);
            static Rotation3D createRotationZJacobian(double radians);

            /**
             * @brief Create a rotation matrix from euler angles
                See: http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
                Computing Euler angles from a rotation matrix
                Gregory G. Slabaugh
                double roll, pitch, yaw; // psi, theta, phi
             */
            static Rotation3D createFromEulerAngles(const arma::vec3& a);

    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_MATRIX_ROTATION3D_H
