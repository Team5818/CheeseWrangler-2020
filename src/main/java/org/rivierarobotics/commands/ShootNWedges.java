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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Flywheel;
import org.rivierarobotics.util.MathUtil;
import org.rivierarobotics.util.VisionTarget;

@GenerateCreator
public class ShootNWedges extends ParallelRaceGroup {

    private static final double TARGET_VEL = 15_900;

    public ShootNWedges(@Provided EjectorCommands ejectorCommands,
                        @Provided VisionCommands visionAimCommands,
                        @Provided CheeseWheelCommands cheeseWheelCommands,
                        @Provided Flywheel flywheel,
                        VisionTarget visionTarget,
                        int wedges) {
        super(
            new CommandBase() {
                @Override
                public void execute() {
                    flywheel.setVelocity(TARGET_VEL);
                }

                @Override
                public void end(boolean interrupted) {
                    flywheel.setVelocity(0);
                }
            },
            new SequentialCommandGroup(
                cheeseWheelCommands.moveToFreeIndex(CheeseWheel.Mode.COLLECT_FRONT, CheeseWheel.Filled.DONT_CARE, 0),
                new ParallelRaceGroup(
                    ejectorCommands.setPower(1.0),
                    new SequentialCommandGroup(
                        new CommandBase() {
                            @Override
                            public boolean isFinished() {
                                return MathUtil.isWithinTolerance(flywheel.getPositionTicks(), TARGET_VEL, 500);
                            }
                        },
                        wedges == 5
                            ? cheeseWheelCommands.all5Shoot().withTimeout(4.0)
                            : cheeseWheelCommands.niceShootinTex(wedges)
                    )
                )
            )
        );
    }
}
