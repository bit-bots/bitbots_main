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

#ifndef UTILITY_MATH_MATRIX_TRANSFORM3D_H
#define UTILITY_MATH_MATRIX_TRANSFORM3D_H

#include <armadillo>

#include "utility/math/matrix/Transform2D.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/math/geometry/UnitQuaternion.h"

namespace utility {
namespace math {
namespace matrix {

    template <int Dimensions>
    class Transform;

    using Transform3D = Transform<3>;

    /**
     * @brief A 4x4 homogeneous orthonormal basis matrix class for representing 3D transformations
     *
     * See:
     * http://en.wikipedia.org/wiki/Transformation_matrix
     * http://en.wikipedia.org/wiki/Rotation_group_SO(3)
     * http://en.wikipedia.org/wiki/Orthogonal_matrix
     * http://en.wikipedia.org/wiki/Orthonormal_basis
     *
     * @author Brendan Annable
     */
    template <>
    class Transform<3> : public arma::mat44 {
        using arma::mat44::mat44; // inherit constructors

        public:
            /**
             * @brief Default constructor creates an identity matrix
             */
            Transform();

            /**
             * @brief Convert from a quaternions vec4
             */
            Transform(const geometry::UnitQuaternion& q);

            /**
             * @brief Convert from a Transform2D matrix
             */
            Transform(const Transform2D& transform);

            /**
             * @brief Convert from a Rotation3D matrix
             */
            Transform(const Rotation3D& rotation);

            /**
             * @brief Convert from a vec6 representing [position_x, position_y, position_z, rotation_x, rotation_y, rotation_z]
             */
            Transform(const arma::vec6& in);

            Transform(const arma::vec3& in);

            /**
             * @brief Translate the current basis by the given 3D vector in local space
             *
             * @param translation The 3D translation vector to translate by
             * @return The transformed basis matrix
             */
            Transform3D translate(const arma::vec3& translation) const;

            /*
             * @brief Translate the current basis along the local X axis
             *
             * This translates along the column vector submatrix(0,0,2,0)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform3D translateX(double translation) const;

            /**
             * @brief Translate the current basis along the local Y axis
             *
             * This translates along the column vector submatrix(0,1,2,1)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform3D translateY(double translation) const;

            /**
             * @brief Translate the current basis along the local Z axis
             *
             * This translates along the column vector submatrix(0,2,2,2)
             *
             * @param translation The amount to translate by
             * @return The transformed basis matrix
             */
            Transform3D translateZ(double translation) const;

            /**
             * @brief Rotates basis matrix around the local X axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform3D rotateX(double radians) const;

            /**
             * @brief Rotates basis matrix around the local Y axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform3D rotateY(double radians) const;

            /**
             * @brief Rotates basis matrix around the local Z axis
             *
             * @param radians The amount to radians to rotate by
             * @return The transformed basis matrix
             */
            Transform3D rotateZ(double radians) const;

            Transform3D rotateLocal(const Rotation3D& rotation, const Transform3D& local) const;
            Transform3D rotateXLocal(double radians, const Transform3D& local) const;
            Transform3D rotateYLocal(double radians, const Transform3D& local) const;
            Transform3D rotateZLocal(double radians, const Transform3D& local) const;

            /**
             * @brief Transforms current basis from world coordinates (i.e. standard basis) to be local to 'reference'
             *
             * @param reference A basis matrix to become relatively local to
             * @return The transformed basis matrix
             */
            Transform3D worldToLocal(const Transform3D& reference) const;

            /**
             * @brief Transforms current basis from local coordinates relative to 'reference', to world coordinates (i.e. standard basis)
             *
             * @param reference The basis matrix that the current basis is relative to
             * @return The transformed basis matrix
             */
            Transform3D localToWorld(const Transform3D& reference) const;

            arma::vec3 transformPoint(const arma::vec3& p);
            arma::vec3 transformVector(const arma::vec3& p);

            /**
             * @brief Performs an orthonormal inverse and returns a new copy
             * Note: Assumes current transform is orthonormal and invertible (which it should be given normal use)
             * Note: Unlike most methods this returns a new transform and does not modify the current transform
             *
             * @return The inverse transform
             */
            Transform3D i() const;

            /**
             * @return The 3x3 rotation matrix
             */
            inline const Rotation3D rotation() const { return submat(0,0,2,2); }
            inline arma::subview<double> rotation() { return submat(0,0,2,2); }

            inline const arma::vec3 translation() const { return submat(0,3,2,3); }
            inline arma::subview<double> translation() { return submat(0,3,2,3); }

            arma::vec3 eulerAngles() const {
                return rotation().eulerAngles();
            }

            /**
             * @brief Creates a translation transform by the given 3D vector
             *
             * @param translation The 3D translation vector to translate by
             * @return The translation transform
             */
            static Transform3D createTranslation(const arma::vec3& translation);

            /**
             * @brief Creates a rotation transform around the X axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform3D createRotationX(double radians);

            /**
             * @brief Creates a rotation transform around the Y axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform3D createRotationY(double radians);

            /**
             * @brief Creates a rotation transform around the Z axis by the given radians
             *
             * @param radians The amount to radians to rotate by
             * @return The rotation transform
             */
            static Transform3D createRotationZ(double radians);
            
            /**
             * @brief Interpolates between two transforms
             *
             * @param alpha
             * @return  alpha * (T2 - T1) + T1;
             */
            static Transform3D interpolate(Transform3D T1, Transform3D T2, float alpha);
    };

}  // matrix
}  // math
}  // utility

#endif  // UTILITY_MATH_MATRIX_TRANSFORM3D_H
