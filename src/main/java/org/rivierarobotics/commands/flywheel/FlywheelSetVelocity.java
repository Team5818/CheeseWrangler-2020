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

package org.rivierarobotics.commands.flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.util.PhysicsUtil;
import org.rivierarobotics.util.ShooterConstants;

@GenerateCreator
public class FlywheelSetVelocity extends CommandBase {
    private final Flywheel flywheel;
    private final PhysicsUtil util;
    private static double vel = 0;

    public FlywheelSetVelocity(@Provided Flywheel flywheel, @Provided PhysicsUtil util) {
        this.flywheel = flywheel;
        this.util = util;
        addRequirements(flywheel);
    }

    public FlywheelSetVelocity(@Provided Flywheel flywheel, double vel) {
        this.flywheel = flywheel;
        FlywheelSetVelocity.vel = vel;
        this.util = null;
        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        if (util != null) {
            vel = ShooterConstants.velocityToTicks(util.getTargetVelocity());
        }
        flywheel.setVelocity(vel);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.setVelocity(0);
    }
}
