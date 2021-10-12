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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.PositionTracker;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class OffsetSixBallTrench extends CommandBase {
    private Command autoCommand;
    private final DriveCommands driveCommands;
    private final VisionCommands visionCommands;
    private final CheeseWheelCommands cheeseWheelCommands;
    private final CollectionCommands collectionCommands;

    public OffsetSixBallTrench(@Provided DriveCommands driveCommands,
                               @Provided VisionCommands visionCommands,
                               @Provided CheeseWheelCommands cheeseWheelCommands,
                               @Provided CollectionCommands collectionCommands) {
        this.visionCommands = visionCommands;
        this.driveCommands = driveCommands;
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.collectionCommands = collectionCommands;
    }

    @Override
    public void initialize() {
        PositionTracker.setPosition(new double[]{-1.7, 3.048});
        this.autoCommand = new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        driveCommands.driveDistance(-2.8, 0.2),
                        new ParallelRaceGroup(
                                collectionCommands.continuous(CheeseWheel.AngleOffset.COLLECT_BACK),
                                new SequentialCommandGroup(
                                        driveCommands.driveDistance(-0.9 * 2 - 0.34, 0.25),
                                        new WaitCommand(0.75)
                                )
                        ),
                        driveCommands.driveDistance(3, 0.6),
                        cheeseWheelCommands.shootUntilEmpty()
                ),
                visionCommands.calcAim(VisionTarget.TOP),
                cheeseWheelCommands.shootUntilEmpty()
        );

        CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public boolean isFinished() {
        return !CommandScheduler.getInstance().isScheduled(autoCommand);
    }
}
