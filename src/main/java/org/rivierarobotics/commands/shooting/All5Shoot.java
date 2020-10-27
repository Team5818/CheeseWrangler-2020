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

package org.rivierarobotics.commands.shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Ejector;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class All5Shoot extends CommandBase {
    private final CheeseWheel cheeseWheel;
    private final Ejector ejector;
    private double start;

    public All5Shoot(@Provided CheeseWheel cheeseWheel, @Provided Ejector ejector) {
        this.cheeseWheel = cheeseWheel;
        this.ejector = ejector;
        addRequirements(cheeseWheel, ejector);
    }

    @Override
    public void initialize() {
        start = cheeseWheel.getPositionTicks();
    }

    @Override
    public void execute() {
        ejector.setPower(1.0);
        cheeseWheel.setPower(0.65);
    }

    @Override
    public void end(boolean interrupted) {
        ejector.setPower(0.0);
        cheeseWheel.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return !MathUtil.isWithinTolerance(cheeseWheel.getPositionTicks(), start, 5096);
    }
}
