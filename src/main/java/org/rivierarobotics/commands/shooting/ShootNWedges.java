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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;
import org.rivierarobotics.subsystems.Turret;
import org.rivierarobotics.util.CheeseSlot;
import org.rivierarobotics.util.MathUtil;

@GenerateCreator
public class ShootNWedges extends SequentialCommandGroup {
    public ShootNWedges(@Provided CheeseWheelCommands cheeseWheelCommands,
                        @Provided EjectorCommands ejectorCommands, @Provided Turret turret,
                        int wedges) {
        for (int i = 0; i < wedges; i++) {
            if(MathUtil.isWithinTolerance(turret.getAngle(), 0, 90)) {
                addCommands(
                        new SequentialCommandGroup(
                                cheeseWheelCommands.cycleSlot(CheeseWheel.Direction.BACKWARDS, CheeseWheel.AngleOffset.SHOOTER_BACK, CheeseSlot.State.BALL),
                                new WaitCommand(0.8),
                                ejectorCommands.setPower(1.0),
                                new WaitCommand(0.5),
                                ejectorCommands.setPower(0)
                        ));
            } else {
                addCommands(
                        new SequentialCommandGroup(
                                cheeseWheelCommands.cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.SHOOTER_FRONT, CheeseSlot.State.BALL),
                                new WaitCommand(0.8),
                                ejectorCommands.setPower(1.0),
                                new WaitCommand(0.5),
                                ejectorCommands.setPower(0)
                        ));
            }
        }
    }
}
