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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.autonomous.AutonomousCommands;
import org.rivierarobotics.autonomous.Pose2dPath;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootLoop extends SequentialCommandGroup {
    public ShootLoop(@Provided AutonomousCommands auto, @Provided CollectionCommands collect,
                     @Provided CheeseWheelCommands shoot, @Provided VisionCommands aim, Pose2dPath loop) {
        super(
                sequence(new WaitCommand(1), shoot.shootNWedges(5))
                    .deadlineWith(aim.encoderAim(VisionTarget.INNER)),
                auto.pathtracer(loop).deadlineWith(collect.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)),
                sequence(new WaitCommand(2), shoot.shootNWedges(5))
                        .deadlineWith(aim.encoderAim(VisionTarget.INNER)),
                shoot.shootNWedges(5)
        );
    }
}
