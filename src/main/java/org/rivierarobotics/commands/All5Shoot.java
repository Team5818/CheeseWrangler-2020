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

package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class All5Shoot extends CommandBase {
    private CheeseWheel cheeseWheel;
    private double halfway;
    private boolean doneHalf;
    private double start;

    public All5Shoot(@Provided CheeseWheel cheeseWheel) {
        this.cheeseWheel = cheeseWheel;
    }

    @Override
    public void initialize() {
        doneHalf = false;
        start = cheeseWheel.getPositionTicks();
        halfway = (start + 2048) % 4096;
    }

    @Override
    public void execute() {
        if (!doneHalf && cheeseWheel.getPositionTicks() > halfway) {
            doneHalf = true;
        }
        cheeseWheel.setManualPower(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        cheeseWheel.setManualPower(0);
    }

    @Override
    public boolean isFinished() {
        var pos = cheeseWheel.getPositionTicks();
        return doneHalf && pos >= start && pos <= halfway;
    }
}
