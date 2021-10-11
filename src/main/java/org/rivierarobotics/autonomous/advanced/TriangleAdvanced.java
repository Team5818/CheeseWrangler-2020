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

package org.rivierarobotics.autonomous.advanced;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.flywheel.FlywheelCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class TriangleAdvanced extends ParallelDeadlineGroup {
    public TriangleAdvanced(@Provided DriveCommands driveCommands, @Provided FlywheelCommands flywheelCommands,
                            @Provided CheeseWheelCommands cheeseWheelCommands, @Provided VisionCommands visionCommands,
                            @Provided CollectionCommands collectionCommands) {
        super(
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                driveCommands.driveDistance(-0.75, -0.3),
                                driveCommands.driveDistance(-0.75, -0.1),
                                driveCommands.rotateTo(90),
                                driveCommands.driveDistance(1, 0.5))),
                                collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)),

                        new SequentialCommandGroup(
                                visionCommands.correctPosition(),
                                visionCommands.calcAim(VisionTarget.TOP),
                                cheeseWheelCommands.shootUntilEmpty()));
    }
}