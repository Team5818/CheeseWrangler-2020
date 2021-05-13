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

package org.rivierarobotics.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Climb;

/**
 * Move the climb to the bottom of the range (ignore safeties) and reset
 * position encoder value. Power defined by constant, no PID. Uses limit
 * switch at bottom to detect end of range.
 */
@GenerateCreator
public class ClimbSetZero extends CommandBase {
    private static final double MAX_POWER = 0.25;
    private final Climb climb;

    public ClimbSetZero(@Provided Climb climb) {
        this.climb = climb;
    }

    @Override
    public void execute() {
        climb.setPower(-MAX_POWER);
    }

    @Override
    public boolean isFinished() {
        return climb.isAtBottom();
    }

    @Override
    public void end(boolean interrupted) {
        climb.resetEncoder();
        climb.setPower(0);
    }
}
