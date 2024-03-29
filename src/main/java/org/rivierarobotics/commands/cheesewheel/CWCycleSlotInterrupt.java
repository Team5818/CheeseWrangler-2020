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
import org.rivierarobotics.util.CheeseSlot;

/**
 * An interrupt for CWCycleSlot. Needed so that CWCycleSlot can be
 * interrupted while moving and set to another slot, or to stop the PID if it
 * is not tuned correctly (oscillation around setpoint). An
 * {@code InstantCommand} that creates and schedules a cycle slot command.
 *
 * @see CWCycleSlot
 */
@GenerateCreator
public class CWCycleSlotInterrupt extends InstantCommand {
    private final CheeseWheel.Direction direction;
    private final CheeseWheel.AngleOffset mode;
    private final CheeseSlot.State requiredState;
    private final CWCycleSlotCreator cwCycleSlot;

    public CWCycleSlotInterrupt(@Provided CWCycleSlotCreator cwCycleSlot, CheeseWheel.Direction direction,
                                CheeseWheel.AngleOffset mode, CheeseSlot.State requiredState) {
        this.direction = direction;
        this.mode = mode;
        this.requiredState = requiredState;
        this.cwCycleSlot = cwCycleSlot;
    }

    @Override
    public void execute() {
        cwCycleSlot.create(direction, mode, requiredState, 0).schedule();
    }
}