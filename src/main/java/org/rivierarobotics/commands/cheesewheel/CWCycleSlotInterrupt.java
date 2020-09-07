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

package org.rivierarobotics.commands.cheesewheel;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;

@GenerateCreator
public class CWCycleSlotInterrupt extends InstantCommand {
    private final CheeseWheel.Direction direction;
    private final CheeseWheel.AngleOffset mode;
    private final boolean requireOpen;
    private final CheeseWheelCommands cheeseWheelCommands;

    public CWCycleSlotInterrupt(@Provided CheeseWheelCommands cheeseWheelCommands, CheeseWheel.Direction direction, CheeseWheel.AngleOffset mode, boolean requireOpen) {
        this.cheeseWheelCommands = cheeseWheelCommands;
        this.direction = direction;
        this.mode = mode;
        this.requireOpen = requireOpen;
    }

    @Override
    public void execute() {
        cheeseWheelCommands.cycleSlot(direction, mode, requireOpen).schedule();
    }
}