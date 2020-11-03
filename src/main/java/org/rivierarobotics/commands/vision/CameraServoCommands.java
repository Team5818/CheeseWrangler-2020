/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.commands.vision;

import org.rivierarobotics.subsystems.ServoPosition;

import javax.inject.Inject;

public class CameraServoCommands {
    private final CameraServoSetPositionCreator setPositionCreator;
    private final CameraImageFlipCreator imageFlipCreator;

    @Inject
    public CameraServoCommands(CameraServoSetPositionCreator setPositionCreator,
                               CameraImageFlipCreator imageFlipCreator) {
        this.setPositionCreator = setPositionCreator;
        this.imageFlipCreator = imageFlipCreator;
    }

    public CameraServoSetPosition setAngle(double angle) {
        return setPositionCreator.create(angle);
    }

    public CameraServoSetPosition setPosition(ServoPosition position) {
        return setPositionCreator.create(position.set);
    }

    public CameraImageFlip flip(boolean setState) {
        return imageFlipCreator.create(setState);
    }

    public CameraImageFlip flip() {
        return imageFlipCreator.create();
    }
}
