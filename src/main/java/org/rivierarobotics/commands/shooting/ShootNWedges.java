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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.commands.cheesewheel.CheeseWheelCommands;
import org.rivierarobotics.commands.ejector.EjectorCommands;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class ShootNWedges extends SequentialCommandGroup {
    public ShootNWedges(@Provided CheeseWheelCommands cheeseWheelCommands,
                        @Provided EjectorCommands ejectorCommands,
                        int wedges) {

        for (int i = 0; i < wedges; i++) {
            addCommands(
                    new ParallelDeadlineGroup(
                            ejectorCommands.setPower(1.0),
                            cheeseWheelCommands.cycleSlot(CheeseWheel.Direction.FORWARDS, CheeseWheel.AngleOffset.COLLECT_FRONT, false)
                    ),
                    ejectorCommands.setPower(0.0)
            );
        }
    }
}
