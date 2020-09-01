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

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.inject.GlobalComponent;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.RobotShuffleboard;
import org.rivierarobotics.util.RobotShuffleboardTab;
import org.rivierarobotics.util.ShooterConstants;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class VisionAim extends ParallelCommandGroup {
    public VisionAim(VisionTarget target, @Provided RobotShuffleboard shuffleboard, @Provided VisionCommands vision) {
        shuffleboard.getTab("Auto Aim").clear();
        shuffleboard.getTab("Auto Aim").setEntry("Aim Mode: ", "Vision Aim");

        Turret.isAbsoluteAngle = true;

        if (target == VisionTarget.TOP) {
            addCommands(vision.autoAimHood(0, ShooterConstants.getTopHeight()),
                vision.autoAimTurret(0, ShooterConstants.getTopHeight()));
        } else {
            addCommands(vision.autoAimHood(ShooterConstants.getDistanceFromOuterToInnerTarget(), ShooterConstants.getTopHeight()),
                vision.autoAimTurret(ShooterConstants.getDistanceFromOuterToInnerTarget(), ShooterConstants.getTopHeight()));
        }
    }
}
