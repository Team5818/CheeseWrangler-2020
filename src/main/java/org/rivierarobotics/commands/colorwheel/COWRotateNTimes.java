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

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.MotionMagicSetPosition;
import org.rivierarobotics.subsystems.ColorWheel;
import org.rivierarobotics.util.RobotShuffleboard;

@GenerateCreator
public class COWRotateNTimes extends MotionMagicSetPosition<ColorWheel> {
    private static final int NUM_COLOR_SLICES_PER_ROTATION = 8;
    private final double rotations;
    private int colorChangeCtr;
    private ColorWheel.GameColor lastColor;

    public COWRotateNTimes(@Provided ColorWheel colorWheel, @Provided RobotShuffleboard shuffleboard, double rotations) {
        super(colorWheel, colorWheel::getPositionTicks, colorWheel::setPositionTicks,
                colorWheel.getPositionTicks() + (rotations * ColorWheel.getTicksPerWheelRevolution()), 50, 10 * rotations, shuffleboard);
        this.rotations = rotations;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0);
    }

    @Override
    public boolean isFinished() {
        boolean finished = super.isFinished();
        ColorWheel.GameColor currentColor = subsystem.getGameColor();

        if (finished && (currentColor == null || colorChangeCtr < rotations * NUM_COLOR_SLICES_PER_ROTATION)) {
            subsystem.setPower(0.1);
            finished = false;
        } else if (lastColor == null || !lastColor.equals(currentColor)) {
            colorChangeCtr++;
            lastColor = currentColor;
        }
        return finished;
    }
}
