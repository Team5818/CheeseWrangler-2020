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

package org.rivierarobotics.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.rivierarobotics.commands.CWCommandGroups;
import org.rivierarobotics.commands.IntakeCommands;
import org.rivierarobotics.commands.LimelightServoCommands;
import org.rivierarobotics.commands.TurretCommands;
import org.rivierarobotics.commands.VisionCommands;
import org.rivierarobotics.util.Side;
import org.rivierarobotics.util.VisionTarget;

import javax.inject.Inject;


public class TrenchRun extends SequentialCommandGroup {

    @Inject
    public TrenchRun(AutonomousCommands autonomousCommands,
                     VisionCommands visionCommands,
                     CWCommandGroups cheesewheelCommandGroups,
                     IntakeCommands intakeCommands,
                     LimelightServoCommands limelightServoCommands,
                     TurretCommands turretCommands) {
        addCommands(
                limelightServoCommands.setAngle(30),
                turretCommands.setAngle(180),
                visionCommands.correctPosition(),
                autonomousCommands.pathweaver(Pose2dPath.MOVETOSHOOT),
                visionCommands.visionAim(VisionTarget.INNER),

                intakeCommands.setPower(Side.BACK),
                autonomousCommands.pathweaver(Pose2dPath.MOVETOCOLLECT),
                cheesewheelCommandGroups.autoCollect(false),

                cheesewheelCommandGroups.autoCollect(false),

                cheesewheelCommandGroups.autoCollect(false),

                autonomousCommands.pathweaver(Pose2dPath.SHOOTCOLLECTED),
                visionCommands.visionAim(VisionTarget.INNER),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBALLL),
                intakeCommands.setPower(Side.FRONT),
                cheesewheelCommandGroups.autoCollect(true),

                autonomousCommands.pathweaver(Pose2dPath.NEXTBALLLL),
                intakeCommands.setPower(Side.FRONT),
                cheesewheelCommandGroups.autoCollect(true),

                autonomousCommands.pathweaver(Pose2dPath.SHOOTCOLLECTEDD));
    }
}