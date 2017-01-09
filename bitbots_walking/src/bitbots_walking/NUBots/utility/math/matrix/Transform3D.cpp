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

#include "Transform3D.h"

namespace utility {
namespace math {
namespace matrix {

    using geometry::UnitQuaternion;

    Transform3D::Transform() {
        eye(); // identity matrix by default
    }

    Transform3D::Transform(const UnitQuaternion& q) : Transform(Rotation3D(q)) {

    }

    Transform3D::Transform(const Rotation3D& rotation) : Transform() {
        submat(0,0,2,2) = rotation;
    }

    Transform3D::Transform(const Transform2D& transform) : Transform(Transform3D().translate({transform.x(), transform.y(), 0}).rotateZ(transform.angle())) {

    }

    Transform3D::Transform(const arma::vec6& in) : Transform(Transform3D().translate(in.rows(0,2)).rotateZ(in[5]).rotateY(in[4]).rotateX(in[3])) {

    }

    Transform3D::Transform(const arma::vec3& in) : Transform(Transform3D().translate(in)) {

    }

    Transform3D Transform3D::translate(const arma::vec3& translation) const {
        return *this * createTranslation(translation);
    }

    Transform3D Transform3D::translateX(double translation) const {
        return translate({translation, 0, 0});
    }

    Transform3D Transform3D::translateY(double translation) const {
        return translate({0, translation, 0});
    }

    Transform3D Transform3D::translateZ(double translation) const {
        return translate({0, 0, translation});
    }

    Transform3D Transform3D::rotateX(double radians) const {
        return *this * createRotationX(radians);
    }

    Transform3D Transform3D::rotateY(double radians) const {
        return *this * createRotationY(radians);
    }

    Transform3D Transform3D::rotateZ(double radians) const {
        return *this * createRotationZ(radians);
    }

    Transform3D Transform3D::rotateLocal(const Rotation3D& rotation, const Transform3D& local) const {
        return Transform3D(Transform3D(rotation) * worldToLocal(local)).localToWorld(local);
    }

    Transform3D Transform3D::rotateXLocal(double radians, const Transform3D& local) const {
        return Transform3D(createRotationX(radians) * worldToLocal(local)).localToWorld(local);
    }

    Transform3D Transform3D::rotateYLocal(double radians, const Transform3D& local) const {
        return Transform3D(createRotationY(radians) * worldToLocal(local)).localToWorld(local);
    }

    Transform3D Transform3D::rotateZLocal(double radians, const Transform3D& local) const {
        return Transform3D(createRotationZ(radians) * worldToLocal(local)).localToWorld(local);
    }

    Transform3D Transform3D::worldToLocal(const Transform3D& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference.i() * (*this);
    }

    Transform3D Transform3D::localToWorld(const Transform3D& reference) const {
        // http://en.wikipedia.org/wiki/Change_of_basis
        return reference * (*this);
    }


    arma::vec3 Transform3D::transformPoint(const arma::vec3& p){
        arma::vec4 p4 = arma::join_cols(p,arma::vec({1}));
        arma::vec4 result4 = *this * p4;
        return result4.rows(0,2);
    }

    arma::vec3 Transform3D::transformVector(const arma::vec3& p){
        arma::vec4 p4 = arma::join_cols(p,arma::vec({0}));
        arma::vec4 result4 = *this * p4;
        return result4.rows(0,2);
    }


    Transform3D Transform3D::i() const {
        // Create a new transform
        Transform3D inverseTransform3D;
        // Transpose the rotation submatrix (top-left 3x3), this is equivalent to taking the inverse of the rotation matrix
        inverseTransform3D.submat(0,0,2,2) = submat(0,0,2,2).t();
        // Multiply translation vector (top-right column vector) by the negated inverse rotation matrix
        inverseTransform3D.submat(0,3,2,3) = -(inverseTransform3D.submat(0,0,2,2) * submat(0,3,2,3));
        /*if (arma::norm(inverseTransform3D * (*this) - arma::eye(4,4)) > 1e-10){
            NUClear::log<NUClear::WARN>("Inverse failed! Matrix is singular");
        }*/
        return inverseTransform3D;
    }

    Transform3D Transform3D::createTranslation(const arma::vec3& translation) {
        Transform3D transform;
        transform.col(3).rows(0,2) = translation;
        return transform;
    }

    Transform3D Transform3D::createRotationX(double radians) {
        Transform3D transform;
        transform.submat(0,0,2,2) = Rotation3D::createRotationX(radians);
        return transform;
    }

    Transform3D Transform3D::createRotationY(double radians) {
        Transform3D transform;
        transform.submat(0,0,2,2) = Rotation3D::createRotationY(radians);
        return transform;
    }

    Transform3D Transform3D::createRotationZ(double radians) {
        Transform3D transform;
        transform.submat(0,0,2,2) = Rotation3D::createRotationZ(radians);
        return transform;
    }

    Transform3D Transform3D::interpolate(Transform3D T1, Transform3D T2, float alpha){
        Rotation3D r1 = T1.rotation();
        UnitQuaternion q1 = UnitQuaternion(r1);
        Rotation3D r2 = T2.rotation();
        UnitQuaternion q2 = UnitQuaternion(r2);

        arma::vec3 t1 = T1.translation();
        arma::vec3 t2 = T2.translation();

        UnitQuaternion qResult = q1.slerp(q2,alpha);
        arma::vec3 tResult = alpha * (t2 - t1) + t1;

        Transform3D TResult = Transform3D(Rotation3D(qResult));
        TResult.translation() = tResult;

        return TResult;
    }

}
}
}
