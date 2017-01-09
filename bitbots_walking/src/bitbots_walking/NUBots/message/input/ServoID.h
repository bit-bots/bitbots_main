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

#ifndef MESSAGE_INPUT_SERVOID_H
#define MESSAGE_INPUT_SERVOID_H

#include <string>

namespace message {
    namespace input {

        enum ServoSide {
            RIGHT = 0,
            LEFT = 1
        };

        enum ServoID {
            //Always right first, this is particular required for idFromPartialString();
            //@Mingze
            R_SHOULDER_PITCH    = 0,
            L_SHOULDER_PITCH    = 1,
            R_SHOULDER_ROLL     = 2,
            L_SHOULDER_ROLL     = 3,
            R_ELBOW             = 4,
            L_ELBOW             = 5,
            R_HIP_YAW           = 6,
            L_HIP_YAW           = 7,
            R_HIP_ROLL          = 8,
            L_HIP_ROLL          = 9,
            R_HIP_PITCH         = 10,
            L_HIP_PITCH         = 11,
            R_KNEE              = 12,
            L_KNEE              = 13,
            R_ANKLE_PITCH       = 14,
            L_ANKLE_PITCH       = 15,
            R_ANKLE_ROLL        = 16,
            L_ANKLE_ROLL        = 17,
            HEAD_YAW            = 18,
            HEAD_PITCH          = 19,
            NUMBER_OF_SERVOS    = 20
        };

        const std::string stringFromId(const ServoID id);
        ServoID idFromString(const std::string str);
        ServoID idFromPartialString(const std::string str, ServoSide side);
    }
}

#endif