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

#ifndef MESSAGE_MOTION_HEADCOMMAND_H
#define MESSAGE_MOTION_HEADCOMMAND_H

#include <armadillo>

namespace message {
    namespace motion {

        /**
         * Tell the head where to look in world space.
         * This command is interpreted such that the robot will use IMU data to fixate at these angles in the world even when rotating.
         *
         * @author Jake Fountain
         */
        struct HeadCommand {
            float yaw;
            float pitch;
            bool robotSpace; // if true, the yaw and pitch are interpreted in robot space, instead of IMU space
        };

    }  // motion
}  // message

#endif  // MESSAGE_MOTION_HEADCOMMAND_H
