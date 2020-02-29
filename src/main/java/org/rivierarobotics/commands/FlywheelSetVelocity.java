/*
 * This file is part of Placeholder-2020, licensed under the GNU General Public License (GPLv3).
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
import org.rivierarobotics.subsystems.Flywheel;

@GenerateCreator
public class FlywheelSetVelocity extends CommandBase {
    private final Flywheel flywheel;
    private final double velocity;

    public FlywheelSetVelocity(@Provided Flywheel flywheel, double velocity) {
        this.flywheel = flywheel;
        this.velocity = velocity;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
    }
}
