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

package org.rivierarobotics.commands.colorwheel;

import org.rivierarobotics.subsystems.ColorWheel;

import javax.inject.Inject;

public class ColorWheelCommands {
    private final COWRotateNTimesCreator cowRotateNTimesCreator;
    private final COWRotateToColorCreator cowRotateToColorCreator;
    private final COWCenterOnColorCreator cowCenterOnColorCreator;

    @Inject
    public ColorWheelCommands(COWRotateNTimesCreator cowRotateNTimesCreator,
                              COWRotateToColorCreator cowRotateToColorCreator,
                              COWCenterOnColorCreator cowCenterOnColorCreator) {
        this.cowRotateNTimesCreator = cowRotateNTimesCreator;
        this.cowRotateToColorCreator = cowRotateToColorCreator;
        this.cowCenterOnColorCreator = cowCenterOnColorCreator;
    }

    public COWRotateNTimes rotateNTimes(double rotations) {
        return cowRotateNTimesCreator.create(rotations);
    }

    public COWRotateToColor rotateToColor(ColorWheel.GameColor color, boolean isField) {
        return cowRotateToColorCreator.create(color, isField);
    }

    public COWRotateToColor rotateToFMS() {
        return cowRotateToColorCreator.create(ColorWheel.GameColor.getFMSColor(), true);
    }

    public COWCenterOnColor centerOnColor() {
        return cowCenterOnColorCreator.create();
    }
}
