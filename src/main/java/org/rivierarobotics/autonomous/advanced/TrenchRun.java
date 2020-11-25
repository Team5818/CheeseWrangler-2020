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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class TrenchRun extends SequentialCommandGroup {
    public TrenchRun(@Provided AutonomousCommands auto, @Provided DriveCommands drive, @Provided CollectionCommands collect,
                     @Provided CheeseWheelCommands shoot, @Provided VisionCommands aim) {
        super(
                sequence(auto.pathtracer(Pose2dPath.START_TOP_TO_SHOOT), shoot.shootNWedges(5))
                        .deadlineWith(aim.encoderAim(VisionTarget.INNER)),
                auto.pathtracer(Pose2dPath.SHOOT_TO_TRENCH_PICKUP),
                drive.driveDistance(1.8, 0.25)
                        .deadlineWith(collect.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)),
                sequence(auto.pathtracer(Pose2dPath.TRENCH_END_TO_SHOOT), shoot.shootNWedges(5))
                        .deadlineWith(aim.encoderAim(VisionTarget.INNER)),
                auto.pathtracer(Pose2dPath.SHOOT_TO_LEFT_CENTER_BALL)
                        .deadlineWith(collect.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)),
                auto.pathtracer(Pose2dPath.SHIFT_LEFT_CENTER_BALL)
                        .deadlineWith(collect.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)),
                sequence(auto.pathtracer(Pose2dPath.LEFT_CENTER_BALL_TO_SHOOT), shoot.shootNWedges(5))
                        .deadlineWith(aim.encoderAim(VisionTarget.INNER))
        );
    }
}
