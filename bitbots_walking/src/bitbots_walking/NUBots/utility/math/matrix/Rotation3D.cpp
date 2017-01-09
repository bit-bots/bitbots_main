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

#include "Rotation3D.h"
#include "matrix.h"
#include "utility/math/comparison.h"
#include "utility/math/angle.h"


namespace utility {
namespace math {
namespace matrix {

    using geometry::UnitQuaternion;
    using utility::math::almost_equal;

    Rotation3D::Rotation() {
        eye(); // identity matrix by default
    }

    Rotation3D::Rotation(const UnitQuaternion& q) {
        // quaternion to rotation conversion
        // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
        // http://en.wikipedia.org/wiki/Rotation_group_SO(3)#Quaternions_of_unit_norm
        *this << 1 - 2 * q.kY() * q.kY() - 2 * q.kZ() * q.kZ() << 2     * q.kX() * q.kY() - 2 * q.kZ() * q.kW() << 2     * q.kX() * q.kZ() + 2 * q.kY() * q.kW() << arma::endr
              << 2     * q.kX() * q.kY() + 2 * q.kZ() * q.kW() << 1 - 2 * q.kX() * q.kX() - 2 * q.kZ() * q.kZ() << 2     * q.kY() * q.kZ() - 2 * q.kX() * q.kW() << arma::endr
              << 2     * q.kX() * q.kZ() - 2 * q.kY() * q.kW() << 2     * q.kY() * q.kZ() + 2 * q.kX() * q.kW() << 1 - 2 * q.kX() * q.kX() - 2 * q.kY() * q.kY();
    }

    Rotation3D::Rotation(const arma::vec3& axis) {
        double normAxis = arma::norm(axis, 2);

        if (normAxis == 0) {
            // Axis has zero length
            eye();
            return;
        }

        // Construct an othonormal basis
        col(0) = axis / normAxis; // x axis
        col(1) = orthonormal(col(0)); // arbitary orthonormal vector
        col(2) = arma::cross(col(0), col(1)); // third othogonal vector
    }

    Rotation3D::Rotation(const arma::vec3& axis, double angle) : Rotation(axis) {
        // Rotate by angle

        *this *= Rotation3D::createRotationX(angle) * i();
    }

    Rotation3D Rotation3D::rotateX(double radians) const {
        return *this * createRotationX(radians);
    }

    Rotation3D Rotation3D::rotateY(double radians) const {
        return *this * createRotationY(radians);
    }

    Rotation3D Rotation3D::rotateZ(double radians) const {
        return *this * createRotationZ(radians);
    }

    Rotation3D Rotation3D::worldToLocal(const Rotation3D& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference.i() * (*this);
    }

    Rotation3D Rotation3D::localToWorld(const Rotation3D& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference * (*this);
    }

    Rotation3D Rotation3D::i() const {
        // http://en.wikipedia.org/wiki/Rotation_matrix#Multiplication
        // The inverse of a rotation matrix is its transpose, which is also a rotation matrix.
        return t();
    }

    AxisAngle Rotation3D::axisAngle() const {
        AxisAngle result;
        arma::cx_vec eigValues;
        arma::cx_mat eigVectors;
        arma::eig_gen(eigValues, eigVectors, *this);

        Axis axis;
        bool axisFound = false;
        for (size_t i = 0; i < eigValues.size(); i++) {
            if (almost_equal(std::real(eigValues[i]), 1.0, 4)) { // account for numeric imprecision
                axis = arma::real(eigVectors.col(i)); // Axis of rotation
                axisFound = true;
            }
        }

        if (arma::norm(axis, 2) == 0 || !axisFound) {
            throw std::domain_error("utility::math::matrix::Rotation3D::axisAngle: No rotation found");
        }

        // Construct an ONB
        arma::vec3 s = orthonormal(axis);
        arma::vec3 t = arma::cross(axis, s);
        // Rotate s to calculate angle of rotation
        arma::vec3 rs = *this * s;

        return {
            axis,
            std::atan2(arma::dot(rs, t), arma::dot(rs, s)) // Angle of rotation
        };
    }

    arma::vec3 Rotation3D::eulerAngles() const {
        // See: http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
        // Computing Euler angles from a rotation matrix
        // Gregory G. Slabaugh
        double roll, pitch, yaw; // psi, theta, phi
        if (!almost_equal(std::abs(at(2,0)), 1.0, 4)) {
            pitch = -utility::math::angle::asin_clamped(at(2,0));
            double cosPitch = std::cos(pitch);
            roll = std::atan2(at(2,1) / cosPitch, at(2,2) / cosPitch);
            yaw = std::atan2(at(1,0) / cosPitch, at(0,0) / cosPitch);
        }
        else {
            roll = std::atan2(at(0,1), at(0,2));
            yaw = 0;
            if (almost_equal(at(2,0), -1.0, 4)) {
                pitch = M_PI_2;
            }
            else {
                pitch = -M_PI_2;
            }
        }
        return {roll, pitch, yaw};
    }


    Rotation3D Rotation3D::createRotationX(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << 1 << 0 <<  0 << arma::endr
                 << 0 << c << -s << arma::endr
                 << 0 << s <<  c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationXJacobian(double radians) {
        double c = -sin(radians);
        double s = cos(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << 1 << 0 <<  0 << arma::endr
                 << 0 << c << -s << arma::endr
                 << 0 << s <<  c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationY(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation <<  c << 0 << s << arma::endr
                 <<  0 << 1 << 0 << arma::endr
                 << -s << 0 << c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationYJacobian(double radians) {
        double c = -sin(radians);
        double s = cos(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation <<  c << 0 << s << arma::endr
                 <<  0 << 1 << 0 << arma::endr
                 << -s << 0 << c;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationZ(double radians) {
        double c = cos(radians);
        double s = sin(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << c << -s << 0 << arma::endr
                 << s <<  c << 0 << arma::endr
                 << 0 <<  0 << 1;
        return rotation;
    }

    Rotation3D Rotation3D::createRotationZJacobian(double radians) {
        double c = -sin(radians);
        double s = cos(radians);
        Rotation rotation;
        // http://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rotation << c << -s << 0 << arma::endr
                 << s <<  c << 0 << arma::endr
                 << 0 <<  0 << 1;
        return rotation;
    }


    Rotation3D Rotation3D::createFromEulerAngles(const arma::vec3& a){
        // double roll = a[0];
        // double pitch = a[1];
        // double yaw = a[2];
        return Rotation3D::createRotationZ(a[2]) * Rotation3D::createRotationY(a[1]) * Rotation3D::createRotationX(a[0]); 
    }

}
}
}