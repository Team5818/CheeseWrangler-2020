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

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.ColorWheel;

@GenerateCreator
public class COWCenterOnColor extends CommandBase {
    private static final int TICKS_PER_HALF_SLICE = 1024;
    private static final double MAX_POWER = 0.1;
    private final ColorWheel colorWheel;
    private ColorWheel.GameColor centerColor;
    private double tickStart;
    private int stage = 0;

    public COWCenterOnColor(@Provided ColorWheel colorWheel) {
        this.colorWheel = colorWheel;
        addRequirements(colorWheel);
    }

    @Override
    public void initialize() {
        centerColor = colorWheel.getGameColor();
    }

    @Override
    public void execute() {
        if (stage == 0) {
            if (!colorWheel.getGameColor().equals(centerColor)) {
                stage++;
                tickStart = colorWheel.getPositionTicks();
            } else {
                colorWheel.setPower(MAX_POWER);
            }
        } else if (stage == 1) {
            if (colorWheel.getPositionTicks() - tickStart >= TICKS_PER_HALF_SLICE) {
                stage++;
            } else {
                colorWheel.setPower(-MAX_POWER);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        colorWheel.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return stage == 2;
    }
}
