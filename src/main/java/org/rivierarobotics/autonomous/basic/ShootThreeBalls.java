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

package org.rivierarobotics.autonomous.basic;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootThreeBalls extends SequentialCommandGroup {
    public ShootThreeBalls(@Provided VisionCommands visionCommands,
                           @Provided CheeseWheelCommands cheeseWheel,
                           @Provided DriveCommands driveCommands) {
        super(
                driveCommands.resetGyro(),
                new WaitCommand(0.5),
                visionCommands.correctPosition(),
                new ParallelDeadlineGroup(
                        new WaitCommand(4).andThen(cheeseWheel.shootUntilEmpty()),
                        visionCommands.calcAim(VisionTarget.TOP)
                ), driveCommands.driveDistance(1.5, 0.5)
        );
    }
}

