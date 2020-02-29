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

import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.CheeseWheel;

import java.util.Objects;

@GenerateCreator
public class CWMoveToFreeIndex extends BasePIDSetPosition<CheeseWheel> {
    private final CheeseWheel.Mode mode;
    private final CheeseWheel.Filled filled;
    private final int direction;

    public CWMoveToFreeIndex(@Provided CheeseWheel cheeseWheel, CheeseWheel.Mode mode, CheeseWheel.Filled filled, int direction) {
        super(cheeseWheel, 20, 0.0);
        this.mode = mode;
        this.filled = filled;
        this.direction = direction;
    }

    private CheeseWheel.Mode getMode() {
        return Objects.requireNonNullElse(mode, subsystem.lastMode);
    }

    @Override
    protected void setSetPosition(double position) {
        positionTicks = subsystem.getClosestSlot(getMode(), filled, 0)
            .next(direction)
            .getModePosition(getMode());
    }
}
