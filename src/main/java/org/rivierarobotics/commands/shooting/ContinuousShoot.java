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

package org.rivierarobotics.commands.shooting;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class ContinuousShoot extends SequentialCommandGroup {
    public ContinuousShoot(@Provided CheeseWheelCommands cheeseWheelCommands,
                           @Provided EjectorCommands ejectorCommands, @Provided Turret turret,
                           @Provided CheeseWheel cheeseWheel) {
        boolean isBack = MathUtil.isWithinTolerance(turret.getAngle(false), 0, 90);
        CheeseWheel.AngleOffset offset = isBack ? CheeseWheel.AngleOffset.SHOOTER_BACK : CheeseWheel.AngleOffset.SHOOTER_FRONT;
        CheeseSlot slot = cheeseWheel.getClosestSlot(offset, offset.direction, CheeseSlot.State.BALL);
        addCommands(
                new ParallelDeadlineGroup(
                        cheeseWheelCommands.cycleSlotWait(offset.direction, offset, CheeseSlot.State.BALL),
                        new WaitCommand(1)
                ),
                new WaitCommand(0.3),
                ejectorCommands.setPower(1),
                new ParallelRaceGroup(
                        new WaitUntilCommand(slot::noBall).andThen(new WaitCommand(0.2)),
                        new WaitCommand(0.5)
                ),
                ejectorCommands.setPower(0),
                new WaitCommand(0.1)
        );
    }
}
