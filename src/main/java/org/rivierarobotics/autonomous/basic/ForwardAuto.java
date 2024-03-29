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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.collect.CollectionCommands;
import org.rivierarobotics.commands.drive.DriveCommands;
import org.rivierarobotics.commands.vision.VisionCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.util.VisionTarget;

/**
 * Basic-type autonomous routine. Does not use PathTracer, not a dynamic auto
 * (i.e. no path selection, no error correction). Shoots, goes forward,
 * collects, shoots again.
 *
 * <p>Process:</p>
 * <ol>
 *     <li>Shoot preloaded balls x5</li>
 *     <li>Continuous side-intelligent collect</li>
 *     <li>Drive 5m back @ 25% pwr, max 6 seconds</li>
 *     <li>Drive 5m forwards @ 25% pwr, max 6 seconds</li>
 *     <li>If useAutoAim: calcAim to inner target for 2s</li>
 *     <li>Shoot collected balls x5</li>
 * </ol>
 */
@GenerateCreator
public class ForwardAuto extends SequentialCommandGroup {
    public ForwardAuto(@Provided DriveCommands drive,
                       @Provided CheeseWheelCommands cheeseWheel,
                       @Provided CollectionCommands intake,
                       @Provided VisionCommands vision,
                       boolean useAutoAim) {
        addCommands(
            cheeseWheel.shootNWedges(5),
            new ParallelDeadlineGroup(
                drive.driveDistance(-5, 0.25).withTimeout(6.0),
                intake.continuous(CheeseWheel.AngleOffset.COLLECT_BACK)
            ),
            new ParallelDeadlineGroup(
                drive.driveDistance(5, 0.5).withTimeout(6.0),
                intake.continuous(CheeseWheel.AngleOffset.COLLECT_FRONT)
            ),
            useAutoAim
                ? vision.calcAim(VisionTarget.INNER).withTimeout(2.0)
                : new InstantCommand(),
            cheeseWheel.shootNWedges(5)
        );
    }
}
