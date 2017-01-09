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

#ifndef UTILITY_MATH_ANGLE_H
#define UTILITY_MATH_ANGLE_H

#include <cmath>
#include <armadillo>

namespace utility {
namespace math {
    /**
     * TODO document
     *
     * @author Trent Houliston
     */
    namespace angle {

        /**
         * Takes an angle in radians and normalizes it to be between -pi and pi
         *
         * @param value the angle in radians
         *
         * @return the angle between -pi and pi
         */
        inline double normalizeAngle(const double value) {

            double angle = std::fmod(value, 2 * M_PI);
            
            if (angle <= -M_PI)
                angle += M_PI * 2;

            if (angle > M_PI)
                angle -= 2 * M_PI;

            return angle;
        }

        inline double acos_clamped(const double& a){
            return std::acos(std::fmax(std::fmin(a,1),-1));
        }

        inline double asin_clamped(const double& a){
            return std::asin(std::fmax(std::fmin(a,1),-1));
        }

        /**
         * Calculates the difference between two angles between -pi and pi
         * Method: http://math.stackexchange.com/questions/1158223/solve-for-x-where-a-sin-x-b-cos-x-c-where-a-b-and-c-are-kno
         * @param
         */
        inline double difference(const double a, const double b) {

            return M_PI - std::fabs(std::fmod(std::fabs(a - b), 2 * M_PI) - M_PI);
        }

        /**
         * Calculates the signed angle with the smallest absolute value such
         * that: normalizeAngle(b + signedDifference(a, b)) == normalizeAngle(a).
         * Method: http://stackoverflow.com/a/7869457
         */
        inline double signedDifference(const double a, const double b) {

            auto x = a - b;

            auto m = x - std::floor(x/(2*M_PI)) * (2*M_PI);

            auto d = std::fmod(m + M_PI, 2*M_PI) - M_PI;

            return d;
        }

        inline double vectorToBearing(arma::vec2 dirVec) {
            return std::atan2(dirVec(1), dirVec(0));
        }

        inline arma::vec2 bearingToUnitVector(double angle) {
            return arma::vec2({std::cos(angle), std::sin(angle)});
        }

        /*! @brief Solves for x in $a \sin(x) + b \cos(x) = c ; x \in [0,\pi]$
        */
        inline float solveLinearTrigEquation(float a, float b, float c){
            float norm = std::sqrt(a*a+b*b);
            if (norm == 0){
                throw std::domain_error("utility::math::angle::solveLinearTrigEquation - std::sqrt(a*a+b*b) == 0 => Any value for x is a solution");
            }

            //Normalise equation
            float a_ = a / norm;
            float b_ = b / norm;
            float c_ = c / norm;

            if(std::fabs(c_) > 1){
                throw std::domain_error("utility::math::angle::solveLinearTrigEquation - no solution |c_|>1");
            }

            //Find alpha such that $\sin(\alpha) = a\_$ and $\cos(\alpha) = b\_$, which is possible because $a\_^2 + b\_^2 = 1$
            float alpha = atan2(a_,b_);

            //Hence the equation becomes $\cos(\alpha)\cos(x)+\sin(\alpha)\sin(x) = cos(x-\alpha) = c\_$
            return alpha + acos_clamped(c_);
        }
    }
}
}
#endif
