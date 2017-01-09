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

#ifndef MESSAGE_INPUT_LIMBID_H
#define MESSAGE_INPUT_LIMBID_H


#include <set>
#include "message/input/ServoID.h"

namespace message {
    namespace input {
        //LimbID is a higher level of ServoID (see ServoID.h),
        //which contains all the constituent servos (e.g. An arm contains shoulder (pitch + roll)) and elbow.

        enum class LimbID {
            LEFT_LEG = 0,
            RIGHT_LEG = 1,
            LEFT_ARM = 2,
            RIGHT_ARM = 3,
            HEAD = 4
        };

        inline std::set<ServoID> servosForLimb(const LimbID& limb) {
            switch(limb) {
                case LimbID::HEAD:
                    return {
                        ServoID::HEAD_PITCH,
                        ServoID::HEAD_YAW
                    };

                case LimbID::LEFT_LEG:
                    return {
                        ServoID::L_ANKLE_PITCH,
                        ServoID::L_ANKLE_ROLL,
                        ServoID::L_HIP_PITCH,
                        ServoID::L_HIP_ROLL,
                        ServoID::L_HIP_YAW,
                        ServoID::L_KNEE
                    };

                case LimbID::RIGHT_LEG:
                    return {
                        ServoID::R_ANKLE_PITCH,
                        ServoID::R_ANKLE_ROLL,
                        ServoID::R_HIP_PITCH,
                        ServoID::R_HIP_ROLL,
                        ServoID::R_HIP_YAW,
                        ServoID::R_KNEE
                    };

                case LimbID::LEFT_ARM:
                    return {
                        ServoID::L_SHOULDER_PITCH,
                        ServoID::L_SHOULDER_ROLL,
                        ServoID::L_ELBOW
                    };

                case LimbID::RIGHT_ARM:
                    return {
                        ServoID::R_SHOULDER_PITCH,
                        ServoID::R_SHOULDER_ROLL,
                        ServoID::R_ELBOW
                    };

                default: {
                    return std::set<ServoID>{};
                }
            }
        }

        inline LimbID limbForServo(const ServoID& servo) {
            switch(servo) {
                case ServoID::HEAD_PITCH:
                case ServoID::HEAD_YAW:
                    return LimbID::HEAD;

                case ServoID::L_ANKLE_PITCH:
                case ServoID::L_ANKLE_ROLL:
                case ServoID::L_HIP_PITCH:
                case ServoID::L_HIP_ROLL:
                case ServoID::L_HIP_YAW:
                case ServoID::L_KNEE:
                    return LimbID::LEFT_LEG;

                case ServoID::R_ANKLE_PITCH:
                case ServoID::R_ANKLE_ROLL:
                case ServoID::R_HIP_PITCH:
                case ServoID::R_HIP_ROLL:
                case ServoID::R_HIP_YAW:
                case ServoID::R_KNEE:
                    return LimbID::RIGHT_LEG;

                case ServoID::L_SHOULDER_PITCH:
                case ServoID::L_SHOULDER_ROLL:
                case ServoID::L_ELBOW:
                    return LimbID::LEFT_ARM;

                case ServoID::R_SHOULDER_PITCH:
                case ServoID::R_SHOULDER_ROLL:
                case ServoID::R_ELBOW:
                    return LimbID::RIGHT_ARM;
                default:
                    // Can't really happen but in case it does make sure someone pays!
                    return static_cast<LimbID>(-1);;
            }
        }
    }
}

#endif

