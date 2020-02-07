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

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import javax.inject.Inject;

public class CWShootIndividual extends SequentialCommandGroup {
    @Inject
    public CWShootIndividual(CheeseWheelCommands cheeseCommands, FlywheelCommands flywheelCommands, EjectorCommands ejectorCommands) {
        addCommands(
                cheeseCommands.setShootMode(true),
                cheeseCommands.setClosestHalfIndex(),
                flywheelCommands.setPower(1.0),
                ejectorCommands.setPower(1.0),
                new WaitCommand(0.5),
                cheeseCommands.incrementIndex(),
                new WaitCommand(0.5),
                cheeseCommands.setShootMode(false),
                cheeseCommands.incrementIndex()
        );
    }
}